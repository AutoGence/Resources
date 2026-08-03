# Handover: AutoGence Resources Documentation Site

Everything needed to run [resources.autogence.ai](https://resources.autogence.ai) on a new
server. Written for someone who has not seen this project before.

**Prepared:** 2026-08-03 · **From commit:** `91ff67d` ("recompile") · **Branch:** `main`

---

## 1. What this actually is

A **static site**. Docusaurus 3.8.1 compiles 32 Markdown pages into plain HTML/CSS/JS
(4.1 MB total) and nginx serves the resulting directory. That's the whole system.

What that means for the migration:

- **No application server, no database, no runtime process.** Nothing to keep alive,
  nothing to port over but files.
- **No environment variables and no secrets in the app.** Verified: zero `process.env`
  references in the entire source tree. There is no `.env` file to hunt down.
- **Search runs in the browser.** `@easyops-cn/docusaurus-search-local` builds
  `search-index.json` at compile time. No Algolia account, no API key, no external
  service to re-point.
- **Node.js is a build-time dependency only.** The production server needs nginx and
  nothing else — it never runs Node.

The only stateful things outside the repo are DNS, the TLS certificate, and four GitHub
Actions secrets. All three are covered below.

---

## 2. What you're receiving

| Item | Where |
|---|---|
| Source repository | `https://github.com/AutoGence/Resources.git` (branch `main`) |
| Source archive | `autogence-resources-src-20260803.tar.gz` |
| Prebuilt site | `autogence-resources-build-20260803.tar.gz` (drop straight into the web root) |
| Checksums | `SHA256SUMS.txt` |

The prebuilt archive was produced from commit `91ff67d` with a clean `npm run build` that
completed with no errors and no broken-link warnings. You can serve it as-is without ever
installing Node — useful for getting the new host answering before you touch anything else.

New files added as part of this handover, in `deploy/`:

- **`nginx-resources.conf`** — the nginx site config. DEPLOYMENT.md has referenced this
  file since October 2025 but it was never actually committed; the config below is
  reconstructed and improved (see §7 for what changed).
- **`setup-server.sh`** — one-time provisioning: packages, web root, nginx, sudoers. Idempotent.
- **`deploy.sh`** — manual build-and-rsync deploy, for the first push to a new host or
  when CI is unavailable.

---

## 3. Fastest path to a working site

Assumes Ubuntu with sudo, and that you are keeping the domain `resources.autogence.ai`.

```bash
# --- on the new server ---
sudo ./deploy/setup-server.sh <deploy-user> resources.autogence.ai

# --- from your machine ---
scp autogence-resources-build-20260803.tar.gz <deploy-user>@<new-host>:/tmp/
ssh <deploy-user>@<new-host> '
  sudo tar -xzf /tmp/autogence-resources-build-20260803.tar.gz \
      -C /var/www/resources.autogence.ai --strip-components=1 &&
  sudo chown -R www-data:www-data /var/www/resources.autogence.ai &&
  sudo systemctl reload nginx'

# --- verify before DNS moves ---
curl -H 'Host: resources.autogence.ai' http://<new-host-ip>/ | head -20
```

That last command is the important one: it proves the new server serves the site correctly
**while the domain still points at the old host**, so you can fix problems with zero
downtime. Only cut DNS over once it returns real HTML.

Then: move DNS (§5), issue TLS (§6), re-point CI (§4).

---

## 4. Re-pointing the deploy pipeline

Deploys are automated by `.github/workflows/deploy.yml`: push to `main` → GitHub Actions
builds with Node 20.18 → rsyncs `build/` to the server over SSH → fixes permissions →
reloads nginx.

To make that pipeline target the new server, generate a fresh deploy key and update four
secrets. **Generate a new key rather than copying the old one** — the old private key is
in the old server's `authorized_keys` and in GitHub's secret store, and rotating during a
migration is free.

```bash
ssh-keygen -t ed25519 -C "github-actions-deploy" -f ~/.ssh/resources_deploy_key
ssh-copy-id -i ~/.ssh/resources_deploy_key.pub <deploy-user>@<new-host>
ssh -i ~/.ssh/resources_deploy_key <deploy-user>@<new-host> 'echo OK'   # must print OK
```

Then at **Settings → Secrets and variables → Actions** on `AutoGence/Resources`:

| Secret | New value |
|---|---|
| `SSH_PRIVATE_KEY` | full contents of `~/.ssh/resources_deploy_key`, `BEGIN`/`END` lines included |
| `REMOTE_HOST` | new server IP or hostname |
| `REMOTE_USER` | the deploy user |
| `REMOTE_TARGET` | `/var/www/resources.autogence.ai` |

`setup-server.sh` already installed the `/etc/sudoers.d/github-deploy` rules the workflow
depends on — without them the permission-fixing steps fail and the job goes red *after*
the files have already synced.

Test with **Actions → Deploy to Production → Run workflow** before relying on it.

> I could not verify the current secret values — they're write-only in GitHub and not in
> the repo. Someone with admin access to the repository has to make these four edits.

---

## 5. DNS

Point `resources.autogence.ai` at the new server's public IP (an `A` record, plus `AAAA`
if you're on IPv6). Lower the TTL to 300s a few hours *before* the cutover so you can
roll back quickly, then raise it again once things are stable.

Keep the old server running and serving for at least 48 hours after the switch. Resolvers
cache aggressively and some clients will keep hitting the old IP well past the stated TTL.

---

## 6. TLS

With DNS resolving to the new host:

```bash
sudo certbot --nginx -d resources.autogence.ai
```

Certbot edits `/etc/nginx/sites-available/resources.autogence.ai` in place, adding the
`:443` block and an HTTP→HTTPS redirect, and installs a renewal timer. Confirm renewal
works:

```bash
sudo certbot renew --dry-run
systemctl list-timers | grep certbot
```

Certbot must run **after** DNS has moved — the HTTP-01 challenge it uses resolves the
domain and expects to reach *this* server.

---

## 7. About the nginx config

`deploy/nginx-resources.conf` differs from the snippet in DEPLOYMENT.md §6 in one way that
matters:

```nginx
# old (from DEPLOYMENT.md)         # new
try_files $uri $uri/ /index.html;  try_files $uri $uri/ =404;
                                   error_page 404 /404.html;
```

The old line is an SPA fallback. Docusaurus is not an SPA — it emits a real directory with
an `index.html` for every route, plus a styled `404.html`. With the fallback in place,
**every mistyped URL returns the homepage with HTTP 200**, which hides broken links from
you and lets search engines index unlimited duplicate pages. The replacement serves the
real 404 page with a real 404 status.

The config also adds gzip, immutable caching for content-hashed `/assets/`, explicit
no-cache on HTML and `search-index.json` (otherwise deploys appear not to land), and
standard security headers.

---

## 8. Verification checklist

After cutover:

- [ ] `https://resources.autogence.ai` loads over HTTPS with a valid certificate
- [ ] `http://` redirects to `https://`
- [ ] Left sidebar navigation renders and expands
- [ ] Right-hand table of contents appears on a docs page
- [ ] **Search returns results** — the most common post-migration breakage; it means
      `search-index.json` is being served correctly
- [ ] A deliberately bad URL (`/docs/does-not-exist`) shows the styled 404 page —
      confirm the status is really 404: `curl -o /dev/null -w '%{http_code}' https://resources.autogence.ai/docs/does-not-exist`
- [ ] Blog index at `/blog` loads
- [ ] Images load (spot-check a page under `/docs/components/actuator-api/`)
- [ ] `curl -I https://resources.autogence.ai/sitemap.xml` returns 200
- [ ] A trivial commit to `main` triggers Actions and the change appears live

---

## 9. Working on the site

```bash
npm ci          # use ci, not install -- respects package-lock.json exactly
npm start       # dev server with hot reload on http://localhost:3001
npm run build   # production build into build/
npm run serve   # preview the production build on :3001
npm run clear   # clear Docusaurus cache when things get strange
```

Port 3001 is deliberate — it avoids colliding with other AutoGence services on 3000.

To add a page: drop a Markdown file into the right `docs/` subdirectory with
`sidebar_position` / `title` / `description` frontmatter. The main sidebar is
autogenerated from the directory tree, so it appears by itself. Only the Quick Start
sidebar is hand-maintained, in `sidebars.js`.

---

## 10. Things worth knowing

**If the domain changes.** One line in `docusaurus.config.js:23` (`url:`). If the site
moves to a subpath rather than a domain root, `baseUrl` on line 25 changes too. Everything
else that mentions the domain is prose in the docs, or the `server_name` and paths in the
nginx config and `deploy.yml`. Nothing is compiled against the hostname except sitemap and
canonical URLs.

**`onBrokenLinks` is set to `'warn'`, not `'throw'`.** Broken internal links will not fail
the build — they'll scroll past in the log. If you want CI to catch them, flip it to
`'throw'` in `docusaurus.config.js:31`, but expect to fix a backlog first.

**`sidebarItemsGenerator.js` at the repo root is dead code.** 281 lines, exports a
function, imported by nothing — leftover from a custom-layout experiment that was reverted
(commits `5ec5efa` / `8b6f198`). It does not affect the build. I left it in place rather
than deleting it during a handover, but it can go.

**`CLAUDE.md` and `.claude/` are gitignored** and therefore absent from the GitHub repo.
`CLAUDE.md` is included in the source archive because it's a genuinely useful description
of the project's conventions.

**Docusaurus 3.9.2 is available** (repo pins 3.8.1). Don't upgrade during the migration —
do the move, confirm it's stable, then upgrade separately so a regression has only one
possible cause.

**`build/` is gitignored** and built fresh by CI on every push. Don't commit it.

**Node versions in play:** `package.json` requires ≥18, CI builds on 20.18, this machine
built the attached artifact on 22.19 — all consistent. Node 20 LTS is the safe choice if
you build anywhere else. The production server needs no Node at all.

---

## 11. Not included

Things I have no access to, which someone at AutoGence must supply:

- The current `SSH_PRIVATE_KEY` / `REMOTE_HOST` / `REMOTE_USER` / `REMOTE_TARGET` values
  (write-only in GitHub; §4 replaces them anyway)
- DNS registrar credentials for `autogence.ai`
- Credentials or access to the old production server
- GitHub admin rights on `AutoGence/Resources`

---

## Reference

- `README.md` — project overview and contribution workflow
- `DEPLOYMENT.md` — original CI/CD setup guide (still accurate apart from the nginx
  `try_files` line discussed in §7)
- `CONTRIBUTING.md` — documentation style and PR guidelines
- `GITHUB_SETUP_CHECKLIST.md` — original repo/server setup checklist
- `CLAUDE.md` — architecture notes and conventions
