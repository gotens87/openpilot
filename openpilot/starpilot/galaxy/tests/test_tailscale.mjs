import assert from 'node:assert/strict'
import { compile } from '../web/vendor/vue/vue.esm-browser.js'
import { GalaxyTailscale, validTailscale } from '../web/js/tailscale.js'

const value = { state: 'offline', busy: false, parked: true, hostname: 'car-node', ips: ['100.64.0.1'],
  auth_url: null, version: '1.104.1', message: null }
assert.equal(validTailscale(value), true)
assert.equal(validTailscale({ ...value, state: 'missing' }), false)
assert.equal(validTailscale({ ...value, busy: 'yes' }), false)
assert.equal(validTailscale({ ...value, auth_url: 'javascript:alert(1)' }), false)
assert.equal(validTailscale({ ...value, auth_url: 'http://login.tailscale.com/a/x' }), false)
assert.equal(validTailscale({ ...value, state: 'needs_login', auth_url: 'https://login.tailscale.com/a/abc' }), true)
assert.equal(validTailscale({ ...value, message: 'x'.repeat(256) }), true)
assert.equal(validTailscale({ ...value, message: 'x'.repeat(257) }), false)
assert.equal((GalaxyTailscale.template.match(/<button\b/g) || []).length, 1)
assert.equal(/uninstall/i.test(GalaxyTailscale.template), false)
assert.ok(GalaxyTailscale.template.includes('Log in to Tailscale'))
assert.ok(GalaxyTailscale.template.includes(':href="loginUrl"'))
assert.ok(GalaxyTailscale.template.includes('needs Internet'))
assert.ok(GalaxyTailscale.template.includes('must be parked'))
assert.ok(GalaxyTailscale.template.includes('!snapshot && !error'))
assert.ok(GalaxyTailscale.template.includes('tone="danger"'))
assert.equal(GalaxyTailscale.props.mode, String)
assert.equal(GalaxyTailscale.props.unauthorized, Function)
compile(GalaxyTailscale.template, { decodeEntities: value => value })

function instance() {
  const vm = { ...GalaxyTailscale.data(), mode: 'local', unauthorized() {} }
  for (const [name, fn] of Object.entries(GalaxyTailscale.methods)) vm[name] = fn.bind(vm)
  for (const [name, fn] of Object.entries(GalaxyTailscale.computed)) Object.defineProperty(vm, name, { get: () => fn.call(vm) })
  return vm
}

const vm = instance()
vm.adopt(value)
assert.equal(vm.statusText, 'Offline')
assert.equal(vm.buttonLabel, 'Get online')
assert.equal(vm.buttonDisabled, false)
assert.equal(vm.pollDelay(), 5000)
vm.adopt({ ...value, state: 'not_installed', hostname: null, ips: [], version: null })
assert.equal(vm.statusText, 'Not installed')
assert.equal(vm.buttonLabel, 'Install Tailscale')
vm.adopt({ ...value, state: 'online', hostname: 'car-node', ips: ['100.64.0.1'] })
assert.equal(vm.statusText, 'Online as car-node · 100.64.0.1')
assert.equal(vm.buttonLabel, 'Repair')
vm.adopt({ ...value, state: 'needs_login', auth_url: 'https://login.tailscale.com/a/abc' })
assert.equal(vm.statusText, 'Needs login')
assert.equal(vm.loginUrl, 'https://login.tailscale.com/a/abc')
assert.equal(vm.buttonLabel, 'Get online')
assert.equal(vm.pollDelay(), 2000)
vm.adopt({ ...value, state: 'starting' })
assert.equal(vm.statusText, 'Starting')
assert.equal(vm.buttonLabel, 'Get online')
assert.equal(vm.buttonDisabled, false)
assert.equal(vm.pollDelay(), 2000)
vm.adopt({ ...value, busy: true })
assert.equal(vm.statusText, 'Working')
assert.equal(vm.buttonLabel, 'Working…')
assert.equal(vm.buttonDisabled, true)
assert.equal(vm.pollDelay(), 2000)
vm.adopt({ ...value, parked: false, busy: false, state: 'offline' })
assert.equal(vm.buttonDisabled, true)
vm.busy = true
assert.equal(vm.buttonLabel, 'Working…')
assert.equal(vm.buttonDisabled, true)
vm.busy = false
assert.throws(() => vm.adopt({ ...value, auth_url: 'javascript:alert(1)' }), /Invalid tailscale status/)
const long = 'e'.repeat(282)
vm.adopt({ ...value, message: long })
assert.equal(vm.snapshot.message, long.slice(0, 256))
assert.equal(vm.buttonDisabled, false)
assert.equal(vm.error, '')

let requests = 0, resolve
globalThis.fetch = () => { requests++; return new Promise(done => { resolve = done }) }
vm.mode = 'sample'; vm.start()
assert.equal(requests, 0)
vm.active = true
const reading = vm.refresh()
vm.stop()
resolve({ ok: true, json: async () => value })
await reading
assert.equal(vm.snapshot, null)
assert.equal(vm.timer, null)
assert.equal(vm.active, false)

const alive = instance()
alive.active = true
alive.timer = setTimeout(() => { throw new Error('poll continued') }, 60000)
GalaxyTailscale.beforeUnmount.call(alive)
assert.equal(alive.active, false)
assert.equal(alive.timer, null)

let auth = 0
const signedOut = instance()
signedOut.unauthorized = () => auth++
signedOut.active = true
globalThis.fetch = async () => ({ ok: false, status: 401, json: async () => ({ error: 'Sign in to Galaxy' }) })
await signedOut.refresh()
assert.equal(auth, 1)
assert.equal(signedOut.active, false)
assert.equal(signedOut.timer, null)
assert.equal(signedOut.error, 'Sign in to Galaxy')

const failed = instance()
failed.adopt(value)
failed.active = true
globalThis.fetch = async () => ({ ok: false, status: 502, json: async () => ({ error: 'Tailscale status unavailable' }) })
await failed.refresh()
assert.equal(failed.error, 'Tailscale status unavailable')
assert.equal(failed.snapshot.state, 'offline')
failed.stop()

const denied = instance()
denied.adopt({ ...value, parked: true, state: 'offline' })
denied.active = true
let posts = 0
denied.unauthorized = () => posts++
globalThis.fetch = async (_url, options) => options?.method === 'POST'
  ? { ok: false, status: 409, json: async () => ({ error: 'Park before changing Tailscale.' }) }
  : { ok: true, json: async () => value }
await denied.connect()
assert.equal(denied.error, 'Park before changing Tailscale.')
assert.equal(posts, 0)
assert.equal(denied.active, true)
denied.stop()

const working = instance()
working.adopt({ ...value, parked: true })
working.active = true
working.unauthorized = () => { throw new Error('unexpected sign-in') }
globalThis.fetch = async (_url, options) => options?.method === 'POST'
  ? { ok: false, status: 409, json: async () => ({ error: 'Tailscale is already working.' }) }
  : { ok: true, json: async () => value }
await working.connect()
assert.equal(working.error, 'Tailscale is already working.')
working.stop()

const posted = instance()
posted.adopt({ ...value, parked: true, state: 'not_installed' })
posted.active = true
let sent
globalThis.fetch = async (url, options) => { sent = { url, method: options?.method, body: options?.body }; return { ok: true, status: 202, json: async () => ({ started: true }) } }
await posted.connect()
assert.match(String(sent.url), /api\/galaxy\/tailscale/)
assert.equal(sent.method, 'POST')
assert.equal(sent.body, JSON.stringify({ action: 'connect' }))
posted.stop()

const blocked = instance()
blocked.active = true
blocked.busy = true
let calls = 0
globalThis.fetch = async () => { calls++; return { ok: true, json: async () => value } }
await blocked.connect()
assert.equal(calls, 0)
blocked.stop()
console.log('Tailscale: schema, Vue template, labels, unmount race, failed reads, auth messages and connect passed')
