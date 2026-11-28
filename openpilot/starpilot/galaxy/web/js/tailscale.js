import { requestJson } from "./startup.js"
import { GxNotice } from "./notice.js"
import { GxState } from "./state.js"

const safeAuthUrl = url => typeof url === 'string' && /^https:\/\/[^\s<>"']+$/.test(url) && url.length <= 2048

export function validTailscale(value) {
  return !!value && ['not_installed', 'offline', 'starting', 'needs_login', 'online'].includes(value.state) &&
    typeof value.busy === 'boolean' && typeof value.parked === 'boolean' &&
    (value.hostname === null || (typeof value.hostname === 'string' && value.hostname.length <= 63)) &&
    Array.isArray(value.ips) && value.ips.length <= 8 && value.ips.every(ip => typeof ip === 'string' && ip.length <= 64) &&
    (value.auth_url === null || safeAuthUrl(value.auth_url)) &&
    (value.version === null || (typeof value.version === 'string' && value.version.length <= 32)) &&
    (value.message === null || (typeof value.message === 'string' && value.message.length <= 256))
}

export const GalaxyTailscale = {
  components: { GxNotice, GxState },
  props: { mode: String, unauthorized: Function },
  data: () => ({ snapshot: null, busy: false, error: '', timer: null, controller: null, generation: 0, active: false }),
  computed: {
    working() { return !!this.snapshot && this.snapshot.busy },
    buttonDisabled() { return this.busy || this.working || !this.snapshot?.parked },
    loginUrl() { return safeAuthUrl(this.snapshot?.auth_url) ? this.snapshot.auth_url : '' },
    statusText() {
      const value = this.snapshot
      if (!value) return ''
      if (value.busy) return 'Working'
      if (value.state === 'online') {
        const parts = [value.hostname, value.ips.filter(Boolean).join(', ')].filter(Boolean)
        return parts.length ? `Online as ${parts.join(' · ')}` : 'Online'
      }
      if (value.state === 'offline') return 'Offline'
      if (value.state === 'needs_login') return 'Needs login'
      if (value.state === 'starting') return 'Starting'
      return 'Not installed'
    },
    buttonLabel() {
      if (this.busy || this.working) return 'Working…'
      if (this.snapshot?.state === 'not_installed') return 'Install Tailscale'
      if (this.snapshot?.state === 'online') return 'Repair'
      return 'Get online'
    },
  },
  mounted() { this.start() },
  beforeUnmount() { this.stop() },
  watch: { mode() { this.stop(); this.start() } },
  methods: {
    start() { if (this.mode === 'local') { this.active = true; this.refresh() } },
    stop() {
      this.active = false; this.generation++; clearTimeout(this.timer); this.timer = null; this.controller?.abort()
      this.snapshot = null; this.busy = false; this.error = ''
    },
    adopt(value) {
      const next = value && typeof value.message === 'string' && value.message.length > 256
        ? { ...value, message: value.message.slice(0, 256) } : value
      if (!validTailscale(next)) throw new Error('Invalid tailscale status')
      this.snapshot = next
    },
    pollDelay() {
      const value = this.snapshot
      return value && (value.busy || value.state === 'starting' || value.state === 'needs_login') ? 2000 : 5000
    },
    showFailure(error) {
      const message = error.message
      if (error.status === 401) { this.stop(); this.unauthorized(); this.error = message; return true }
      this.error = message
      return false
    },
    async refresh() {
      if (!this.active || this.busy) return
      clearTimeout(this.timer); this.controller?.abort()
      const generation = ++this.generation
      this.controller = new AbortController()
      try {
        const value = await requestJson('./api/galaxy/tailscale', { request: { signal: this.controller.signal } })
        if (!this.active || generation !== this.generation) return
        this.adopt(value); this.error = ''
      } catch (error) {
        if (!this.active || generation !== this.generation) return
        if (this.showFailure(error)) return
      } finally {
        if (this.active && generation === this.generation) this.timer = setTimeout(() => this.refresh(), this.pollDelay())
      }
    },
    async connect() {
      if (!this.active || this.busy) return
      clearTimeout(this.timer); this.controller?.abort()
      const generation = ++this.generation
      this.controller = new AbortController(); this.busy = true; this.error = ''
      try {
        await requestJson('./api/galaxy/tailscale', { request: { signal: this.controller.signal,
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action: 'connect' }),
        } })
        if (!this.active || generation !== this.generation) return
      } catch (error) {
        if (!this.active || generation !== this.generation) return
        if (this.showFailure(error)) return
      } finally {
        if (this.active && generation === this.generation) { this.busy = false; this.timer = setTimeout(() => this.refresh(), 1000) }
      }
    },
  },
  template: `<section class="gx-card gx-panel gx-stack"><div class="gx-section__header"><i class="bi bi-shield-lock" aria-hidden="true"></i><span class="gx-section__title">Tailscale</span></div>
    <p>Brings this car onto your Tailscale network so you can reach it from your phone. Installing binaries needs Internet. The car must be parked.</p>
    <GxState v-if="mode !== 'local'">Connect to the car to manage Tailscale.</GxState>
    <GxState v-else-if="!snapshot && !error" loading>Loading Tailscale status…</GxState>
    <template v-if="snapshot">
      <GxNotice :busy="working" :tone="snapshot.state === 'offline' || snapshot.state === 'needs_login' ? 'warn' : 'info'">{{ statusText }}</GxNotice>
      <p v-if="snapshot.version" class="gx-note">Version {{ snapshot.version }}</p>
      <div class="gx-actions"><button class="gx-btn" type="button" :disabled="buttonDisabled" @click="connect">{{ buttonLabel }}</button>
        <a v-if="loginUrl" class="gx-btn gx-btn--tonal" :href="loginUrl" target="_blank" rel="noopener">Log in to Tailscale</a></div>
      <GxNotice v-if="!snapshot.parked">Park before changing Tailscale.</GxNotice>
      <GxNotice v-if="snapshot.message">{{ snapshot.message }}</GxNotice>
    </template>
    <GxNotice v-if="error" tone="danger">{{ error }}</GxNotice>
  </section>`,
}
