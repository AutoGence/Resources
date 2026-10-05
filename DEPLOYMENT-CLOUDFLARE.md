# Deploying resources.autogence.ai

`resources.autogence.ai` is a **Cloudflare Worker serving static assets**. There is no
origin web server in the path. This repo holds Docusaurus *source*, so publishing
always requires a build — copying the repo somewhere is not a deployment.

| | |
|---|---|
| Worker | `autogence-resources` |
| Route | `resources.autogence.ai` (custom domain) |
| Worker script | `worker/index.js` (in this repo) |
| Worker config | `wrangler.jsonc` (in this repo) |
| Assets | `build/`, produced by `npm run build` |

## Normal deployment

Push to `main`. `.github/workflows/deploy.yml` builds the site and publishes the Worker
with `wrangler`. Nothing else is required.

### Required repository secrets

| secret | purpose |
|---|---|
| `CLOUDFLARE_API_TOKEN` | authorises the deploy |
| `CLOUDFLARE_ACCOUNT_ID` | which Cloudflare account to publish to |

**Scope the token to this Worker.** A token with blanket Workers access would let
anything that compromises this workflow publish to *every* Worker on the account —
and this repository is public. Cloudflare's "Edit Cloudflare Workers" template,
restricted to this account, is the right level.

## What changed, and why it was broken

Until 2026-10-04 the workflow built the site and rsynced it over SSH to a server, then
reloaded nginx. **It reported success on every push and did not update the live site**,
because the site had already moved to a Cloudflare Worker.

Measured: the run for commit `7773f9d` ("Add HARFY software and system services page")
completed **success in 3m37s at 04:11:58Z**, and minutes later
`https://resources.autogence.ai/harfy-services/` was still serving the homepage. The
page only went live when it was published to the Worker by hand.

The Worker's script and config also lived **only on the deploy host**, never in this
repo, so CI could not have deployed the Worker even if it had tried. Both now live
here — that is what makes the workflow above possible.

The old SSH secrets (`REMOTE_HOST`, `REMOTE_USER`, `REMOTE_TARGET`, `SSH_PRIVATE_KEY`)
are no longer used by this workflow. Remove them once you are satisfied nothing else
depends on them.

## Verifying a deploy — check content, not status codes

**The Worker returns HTTP 200 for unknown paths.** It reproduces nginx's
`try_files $uri $uri/ /index.html`, so a page that does not exist falls through to the
homepage *with a 200*. A status check will report a missing page as healthy.

```bash
curl -s https://resources.autogence.ai/harfy-services/ -o /tmp/page.html
curl -s https://resources.autogence.ai/              -o /tmp/home.html
cmp -s /tmp/page.html /tmp/home.html && echo "NOT DEPLOYED (fallback)" || echo "deployed"
```

Other behaviour worth confirming, all measured on 2026-10-04:

| check | expected |
|---|---|
| `/favicon.ico` | 204 |
| `/README.md` | 403 (denied) |
| `/node_modules/x` | 403 |
| `/harfy-services` (no slash) | 301 → `/harfy-services/` |
| `/search-index.json` | **200** — denying it breaks the search box |
| unknown path | 200, serving the homepage |

## Manual deployment, if CI is unavailable

From a checkout, with `CLOUDFLARE_API_TOKEN` and `CLOUDFLARE_ACCOUNT_ID` exported:

```bash
npm ci --no-audit --no-fund
npm run build
npx wrangler deploy
```

## Known behaviour that looks like a bug and is not

**Cloudflare serves static assets before the Worker runs**, so any Worker rule aimed at
a path that *exists as a file* never fires:

- `/.nojekyll` returns 200, not the 403 the deny rule intends — the file exists, so the
  asset layer serves it. `/README.md` returns 403 only because it is absent from the
  build and therefore reaches the Worker.
- Hashed assets are served `max-age=0, must-revalidate` rather than the intended
  one-year immutable cache, for the same reason.

The second is a real performance loss. Fixing it needs `run_worker_first`, which makes
every asset request a billable Worker invocation — a deliberate trade, not a change to
make casually.

## Stale assets

Docusaurus emits content-hashed filenames. The Worker bundle had accumulated **288
files where the build produces 153** — five `styles.*.css` and three copies of several
JS chunks across past builds — because older deployments copied without deleting.
`wrangler deploy` uploads exactly what is in `build/`, so this no longer accumulates.

## Rollback

`wrangler deployments list`, then roll back to the previous version id. Rebuilding from
the previous commit and redeploying also works.

---
Written 2026-10-04. Every claim measured on the `7773f9d` deploy, not inferred.
