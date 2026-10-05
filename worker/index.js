// Faithful port of the nginx vhost that served resources.autogence.ai.
// Ported rules, in nginx's matching order (exact `=`, then regex in order):
//
//   location = /favicon.ico                      -> 204
//   location ~* \.(js|css|png|...|eot)$          -> Cache-Control 1y immutable
//   location ~ /\.                               -> deny (403)
//   location ~* \.(md|json|lock)$                -> deny (403)
//   location ~ /node_modules                     -> deny (403)
//   location /  { try_files $uri $uri/ /index.html;
//                 location ~* \.html$ -> Cache-Control 1h }
//   plus the three always-on security headers.
//
// TWO DELIBERATE DEVIATIONS from the nginx original, both fixing defects:
//
// 1. `json` is NOT denied. nginx's `location ~* \.(md|json|lock)$ { deny all; }`
//    also blocked Docusaurus' own search-index.json / search-doc.json /
//    lunr-index-*.json, which is what its search box fetches - so search was
//    broken in production. Re-add `json` to DENY_EXT to restore the 403s.
//
// 2. The security headers are sent on HTML too. nginx declared them at server
//    level, but the inner `location ~* \.html$` block carries its own
//    add_header, and nginx does not inherit add_header into a block that
//    defines any - so HTML pages went out with no X-Frame-Options, no
//    nosniff and no CSP. The CSP allows `https:` broadly, so applying it is
//    very unlikely to block anything the site loads.

const STATIC_EXT = /\.(js|css|png|jpg|jpeg|gif|ico|svg|woff|woff2|ttf|eot)$/i;
const DENY_EXT = /\.(md|lock)$/i;   // `json` deliberately dropped - see note above

const SECURITY_HEADERS = {
  'X-Frame-Options': 'SAMEORIGIN',
  'X-Content-Type-Options': 'nosniff',
  'Content-Security-Policy':
    "default-src 'self' https: data: blob: 'unsafe-inline' fonts.googleapis.com fonts.gstatic.com",
};

function decorate(res, cacheControl) {
  const h = new Headers(res.headers);
  for (const [k, v] of Object.entries(SECURITY_HEADERS)) h.set(k, v);
  if (cacheControl) h.set('Cache-Control', cacheControl);
  return new Response(res.body, { status: res.status, statusText: res.statusText, headers: h });
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const path = url.pathname;

    if (path === '/favicon.ico') return decorate(new Response(null, { status: 204 }));

    if (/\/\./.test(path) || DENY_EXT.test(path) || path.includes('/node_modules')) {
      return decorate(new Response(null, { status: 403 }));
    }

    const direct = await env.ASSETS.fetch(request);
    if (direct.status !== 404) {
      const cc = STATIC_EXT.test(path)
        ? 'public, immutable, max-age=31536000'
        : path.endsWith('.html') ? 'public, max-age=3600' : null;
      return decorate(direct, cc);
    }

    // try_files $uri/ — nginx 301s to add the trailing slash before serving the index
    if (!path.endsWith('/') && !path.includes('.')) {
      const probe = await env.ASSETS.fetch(new Request(new URL(path + '/index.html', url), request));
      if (probe.status !== 404) {
        return decorate(Response.redirect(url.origin + path + '/' + url.search, 301));
      }
    }
    if (path.endsWith('/')) {
      const index = await env.ASSETS.fetch(new Request(new URL(path + 'index.html', url), request));
      if (index.status !== 404) return decorate(index, 'public, max-age=3600');
    }

    // try_files ... /index.html — unknown paths fall through to the root index with 200
    const root = await env.ASSETS.fetch(new Request(new URL('/index.html', url), request));
    return root.status !== 404 ? decorate(root, 'public, max-age=3600') : decorate(direct);
  },
};
