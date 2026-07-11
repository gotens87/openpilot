import { html, reactive } from "/assets/vendor/arrow-core.js"

const MAX_RENDERED_ROUTES = 250
const ROUTE_FLUSH_INTERVAL_MS = 120
const STATUS_POLL_MS = 3000

const state = reactive({
  loadingRoutes: true,
  loadingWorkspace: true,
  runningAction: false,
  error: "",
  routes: [],
  selectedRoutes: [],
  truncatedRoutes: false,
  routeProgress: 0,
  routeTotal: 0,
  workspace: { reports: [], activeTrial: null, status: {} },
  status: {},
  report: null,
  feedbackAccepted: [],
  feedbackIgnored: [],
  feedbackNotes: "",
})

let routesAbortController = null
let routesRequestToken = 0
let pendingRoutes = []
let flushTimerId = null
let seenRouteNames = new Set()
let initialized = false
let statusPollHandle = null

function isTuningRouteActive() {
  return window.location.pathname === "/tuning" || window.location.pathname === "/lateral_maneuvers"
}

function formatTimestamp(value) {
  if (!value) return "Unknown Route"
  const parsed = new Date(value)
  if (Number.isNaN(parsed.getTime())) return String(value)
  return parsed.toLocaleString()
}

function safeCount(value) {
  const n = Number(value)
  return Number.isFinite(n) ? n : 0
}

function formatStatusAge(updatedAt) {
  const updated = Number(updatedAt)
  if (!Number.isFinite(updated) || updated <= 0) return "unknown"
  const ageSec = Math.max(0, Math.round(Date.now() / 1000 - updated))
  if (ageSec < 5) return "just now"
  if (ageSec < 60) return `${ageSec}s ago`
  if (ageSec < 3600) return `${Math.round(ageSec / 60)}m ago`
  return `${Math.round(ageSec / 3600)}h ago`
}

function resetRouteStreamState() {
  pendingRoutes = []
  seenRouteNames = new Set()
  if (flushTimerId !== null) {
    clearTimeout(flushTimerId)
    flushTimerId = null
  }
}

function sortedRoutes() {
  return [...state.routes].sort((a, b) => {
    const aTime = Date.parse(a.timestamp)
    const bTime = Date.parse(b.timestamp)
    if (Number.isFinite(aTime) && Number.isFinite(bTime)) return bTime - aTime
    return String(b.timestamp || "").localeCompare(String(a.timestamp || ""))
  })
}

function flushPendingRoutes() {
  if (!pendingRoutes.length) return

  const availableSlots = Math.max(MAX_RENDERED_ROUTES - state.routes.length, 0)
  if (availableSlots <= 0) {
    pendingRoutes = []
    state.truncatedRoutes = true
    return
  }

  const toAppend = pendingRoutes.slice(0, availableSlots)
  pendingRoutes = []
  if (toAppend.length > 0) {
    state.routes = [...state.routes, ...toAppend]
  }
  if (state.routes.length >= MAX_RENDERED_ROUTES) {
    state.truncatedRoutes = true
  }
}

function enqueueRoutes(rawRoutes) {
  if (!Array.isArray(rawRoutes) || rawRoutes.length === 0) return
  const nextRoutes = []
  for (const route of rawRoutes) {
    const name = String(route?.name || "")
    if (!name || seenRouteNames.has(name)) continue
    seenRouteNames.add(name)
    nextRoutes.push({
      ...route,
      timestampLabel: formatTimestamp(route.timestamp),
    })
  }
  if (!nextRoutes.length) return

  pendingRoutes.push(...nextRoutes)
  if (flushTimerId === null) {
    flushTimerId = setTimeout(() => {
      flushTimerId = null
      flushPendingRoutes()
    }, ROUTE_FLUSH_INTERVAL_MS)
  }
}

async function fetchRoutes() {
  const requestToken = ++routesRequestToken
  if (routesAbortController) routesAbortController.abort()
  const controller = new AbortController()
  routesAbortController = controller

  try {
    state.loadingRoutes = true
    state.error = ""
    const userTimezone = Intl.DateTimeFormat().resolvedOptions().timeZone
    const response = await fetch(`/api/routes?timezone=${encodeURIComponent(userTimezone)}`, { signal: controller.signal })
    if (!response.ok) throw new Error("Failed to load local routes.")

    const reader = response.body.getReader()
    const decoder = new TextDecoder()
    let buffer = ""

    while (true) {
      const { value, done } = await reader.read()
      if (done) break
      if (requestToken !== routesRequestToken) return

      buffer += decoder.decode(value, { stream: true })
      const lines = buffer.split(/\r?\n\r?\n/)
      buffer = lines.pop() || ""

      for (const line of lines) {
        if (!line.startsWith("data:")) continue
        try {
          const data = JSON.parse(line.substring(5).trim())
          if (data.progress !== undefined && data.total !== undefined) {
            state.routeProgress = safeCount(data.progress)
            state.routeTotal = safeCount(data.total)
          }
          if (Array.isArray(data.routes)) {
            enqueueRoutes(data.routes)
          }
        } catch (error) {
          console.error("[ftm] failed to parse route payload", error)
        }
      }
    }

    flushPendingRoutes()
  } catch (error) {
    if (error?.name !== "AbortError") {
      state.error = error?.message || "Failed to load local routes."
    }
  } finally {
    if (requestToken === routesRequestToken) {
      flushPendingRoutes()
      state.loadingRoutes = false
      if (routesAbortController === controller) {
        routesAbortController = null
      }
    }
  }
}

async function fetchWorkspace() {
  try {
    state.loadingWorkspace = true
    const response = await fetch("/api/ftm/workspace")
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to load tuning workspace.")
    state.workspace = payload
    state.status = payload.status || {}
  } catch (error) {
    state.error = error?.message || "Failed to load tuning workspace."
  } finally {
    state.loadingWorkspace = false
  }
}

function syncFeedbackState(report) {
  const feedback = report?.feedback || {}
  state.feedbackAccepted = Array.isArray(feedback.acceptedDimensions) ? [...feedback.acceptedDimensions] : []
  state.feedbackIgnored = Array.isArray(feedback.ignoredDimensions) ? [...feedback.ignoredDimensions] : []
  state.feedbackNotes = typeof feedback.notes === "string" ? feedback.notes : ""
}

async function loadReport(reportId) {
  if (!reportId) return
  try {
    const response = await fetch(`/api/ftm/report/${encodeURIComponent(reportId)}`)
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to load tuning report.")
    state.report = payload
    syncFeedbackState(payload)
    await fetchWorkspace()
  } catch (error) {
    state.error = error?.message || "Failed to load tuning report."
  }
}

async function deleteReport(reportId) {
  if (!reportId || state.runningAction) return
  if (!window.confirm("Delete this saved tuning report and its generated trial data?")) return

  state.runningAction = true
  try {
    const response = await fetch(`/api/ftm/report/${encodeURIComponent(reportId)}`, { method: "DELETE" })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to delete tuning report.")

    if (state.report?.reportId === reportId) {
      state.report = null
      syncFeedbackState(null)
    }
    state.workspace = payload.workspace || state.workspace
    state.status = { ...state.status, ...(payload.workspace?.status || {}) }
    showSnackbar(payload.message || "Deleted tuning report.")
  } catch (error) {
    state.error = error?.message || "Failed to delete tuning report."
    showSnackbar(state.error, "error")
  } finally {
    state.runningAction = false
  }
}

async function clearWorkspace() {
  if (state.runningAction) return
  if (!window.confirm("Clear every saved tuning report, feedback entry, generated profile, and snapshot from the device?")) return

  state.runningAction = true
  try {
    const response = await fetch("/api/ftm/workspace/clear", { method: "POST" })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to clear tuning workspace.")

    state.report = null
    syncFeedbackState(null)
    state.workspace = payload.workspace || { reports: [], activeTrial: null, status: {} }
    state.status = { ...state.status, ...(payload.workspace?.status || {}) }
    showSnackbar(payload.message || "Cleared tuning workspace.")
  } catch (error) {
    state.error = error?.message || "Failed to clear tuning workspace."
    showSnackbar(state.error, "error")
  } finally {
    state.runningAction = false
  }
}

async function fetchStatus() {
  try {
    const response = await fetch("/api/ftm/status")
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to load tuning status.")
    state.status = {
      ...(payload.status || {}),
      isOnroad: !!payload.isOnroad,
    }
    if (payload.activeTrial !== undefined) {
      state.workspace = { ...state.workspace, activeTrial: payload.activeTrial, reports: payload.reports || state.workspace.reports }
    }
    const reportId = state.status.reportId
    if (reportId && state.report?.reportId !== reportId) {
      await loadReport(reportId)
    }
  } catch (error) {
    state.error = error?.message || "Failed to load tuning status."
  }
}

function stopPolling() {
  if (statusPollHandle) {
    clearTimeout(statusPollHandle)
    statusPollHandle = null
  }
}

function ensurePolling() {
  if (statusPollHandle) return

  const poll = async () => {
    if (!isTuningRouteActive()) {
      stopPolling()
      return
    }
    if (document.visibilityState === "visible") {
      await fetchStatus()
    }
    statusPollHandle = setTimeout(poll, STATUS_POLL_MS)
  }

  statusPollHandle = setTimeout(poll, STATUS_POLL_MS)
}

function toggleRouteSelection(routeName) {
  const current = new Set(state.selectedRoutes)
  if (current.has(routeName)) {
    current.delete(routeName)
  } else {
    current.add(routeName)
  }
  state.selectedRoutes = [...current]
}

function clearSelections() {
  state.selectedRoutes = []
}

async function runAnalyze() {
  if (!state.selectedRoutes.length || state.runningAction) return
  state.runningAction = true
  try {
    const response = await fetch("/api/ftm/analyze", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ routes: state.selectedRoutes }),
    })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to start tuning analysis.")
    state.status = payload.status || {}
    showSnackbar(payload.message || "FTM analysis started.")
  } catch (error) {
    state.error = error?.message || "Failed to start tuning analysis."
    showSnackbar(state.error, "error")
  } finally {
    state.runningAction = false
  }
}

async function stopAnalyze() {
  if (state.runningAction) return
  state.runningAction = true
  try {
    const response = await fetch("/api/ftm/analyze/stop", { method: "POST" })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to stop tuning analysis.")
    state.status = payload.status || {}
    showSnackbar(payload.message || "FTM analysis stopped.")
  } catch (error) {
    state.error = error?.message || "Failed to stop tuning analysis."
    showSnackbar(state.error, "error")
  } finally {
    state.runningAction = false
  }
}

async function applyProfile(profileId) {
  if (!state.report?.reportId || !profileId || state.runningAction) return
  state.runningAction = true
  try {
    const response = await fetch("/api/ftm/trials/apply", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ reportId: state.report.reportId, profileId }),
    })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to apply trial profile.")
    await fetchWorkspace()
    showSnackbar(payload.message || "Trial profile applied.")
  } catch (error) {
    state.error = error?.message || "Failed to apply trial profile."
    showSnackbar(state.error, "error")
  } finally {
    state.runningAction = false
  }
}

async function selectPath(pathKey) {
  if (!state.report?.reportId || !pathKey || state.runningAction) return
  if (pathKey === (state.report.selectedPathKey || state.report.primaryPathKey)) return

  state.runningAction = true
  try {
    const response = await fetch(`/api/ftm/report/${encodeURIComponent(state.report.reportId)}/path`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ pathKey }),
    })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to select tuning path.")
    state.report = payload.report
    syncFeedbackState(state.report)
    showSnackbar(payload.message || "Tuning path selected.")
  } catch (error) {
    state.error = error?.message || "Failed to select tuning path."
    showSnackbar(state.error, "error")
  } finally {
    state.runningAction = false
  }
}

async function revertProfile() {
  if (state.runningAction) return
  state.runningAction = true
  try {
    const response = await fetch("/api/ftm/trials/revert", { method: "POST" })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to revert trial profile.")
    await fetchWorkspace()
    showSnackbar(payload.message || "Trial profile reverted.")
  } catch (error) {
    state.error = error?.message || "Failed to revert trial profile."
    showSnackbar(state.error, "error")
  } finally {
    state.runningAction = false
  }
}

function setDimensionFeedback(dimensionId, mode) {
  const accepted = new Set(state.feedbackAccepted)
  const ignored = new Set(state.feedbackIgnored)
  accepted.delete(dimensionId)
  ignored.delete(dimensionId)
  if (mode === "accepted") accepted.add(dimensionId)
  if (mode === "ignored") ignored.add(dimensionId)
  state.feedbackAccepted = [...accepted]
  state.feedbackIgnored = [...ignored]
}

async function saveFeedback() {
  if (!state.report?.reportId || state.runningAction) return
  state.runningAction = true
  try {
    const response = await fetch("/api/ftm/feedback", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        reportId: state.report.reportId,
        acceptedDimensions: state.feedbackAccepted,
        ignoredDimensions: state.feedbackIgnored,
        notes: state.feedbackNotes,
      }),
    })
    const payload = await response.json()
    if (!response.ok) throw new Error(payload.error || "Failed to save tuning feedback.")
    state.report = payload.report || {
      ...state.report,
      feedback: payload.feedback,
      profiles: payload.profiles || state.report.profiles,
    }
    syncFeedbackState(state.report)
    await fetchWorkspace()
    showSnackbar(payload.message || "Feedback saved.")
  } catch (error) {
    state.error = error?.message || "Failed to save tuning feedback."
    showSnackbar(state.error, "error")
  } finally {
    state.runningAction = false
  }
}

function feedbackStateFor(dimensionId) {
  if (state.feedbackAccepted.includes(dimensionId)) return "accepted"
  if (state.feedbackIgnored.includes(dimensionId)) return "ignored"
  return "unset"
}

function initialize() {
  if (initialized) return
  initialized = true
  resetRouteStreamState()
  fetchRoutes()
  fetchWorkspace().then(() => {
    const latestReport = state.workspace?.reports?.[0]?.reportId
    if (latestReport) loadReport(latestReport)
  })
  fetchStatus()
  ensurePolling()
}

function renderCurve(values) {
  return `[${(values || []).map((value) => Number(value).toFixed(3)).join(", ")}]`
}

function reportPaths() {
  if (Array.isArray(state.report?.paths) && state.report.paths.length) {
    return state.report.paths
  }
  if (!state.report) return []
  return [{
    key: state.report.primaryPathKey || "cleanup_pass",
    title: "Recommendations",
    description: "",
    whenToUse: "",
    whySelected: "",
    isPrimary: true,
    suggestions: state.report.suggestions || [],
    profiles: state.report.profiles || [],
  }]
}

function primaryPath() {
  const paths = reportPaths()
  const selectedPathKey = state.report?.selectedPathKey || state.report?.primaryPathKey
  return paths.find((path) => path.key === selectedPathKey) || paths.find((path) => path.isPrimary) || paths[0] || null
}

function renderProfile(profile) {
  const genericEntries = Object.entries(profile.genericParams || {}).filter(([key]) => key !== "AdvancedLateralTune")
  const frictionEntries = Object.entries(profile.ftmOverrides?.baseFrictionThresholds || {})
  const vehicleKnobEntries = Object.entries(profile.ftmOverrides?.vehicleKnobs || {})
  return html`
    <div class="ftmCard">
      <div class="ftmCardHeader">
        <div>
          <h4>${profile.label}</h4>
          <p class="longManeuverMuted">${profile.description}</p>
        </div>
        <button
          class="longManeuverButton"
          disabled="${() => state.runningAction || false}"
          @click="${() => applyProfile(profile.id)}">
          Apply Trial
        </button>
      </div>

      <div class="ftmProfileGrid">
        <div>
          <h5>Generic Params</h5>
          <ul>
            ${genericEntries.length
              ? genericEntries.map(([key, value]) => html`<li><code>${key}</code>: ${String(value)}</li>`)
              : html`<li>None</li>`}
          </ul>
        </div>
        <div>
          <h5>FTM Overrides</h5>
          <ul>
            ${frictionEntries.map(([family, payload]) => html`<li><code>${family}</code>: ${renderCurve(payload?.values || [])}</li>`)}
            ${vehicleKnobEntries.map(([key, value]) => html`<li><code>${key}</code>: ${Number(value).toFixed(3)}</li>`)}
            ${!frictionEntries.length && !vehicleKnobEntries.length ? html`<li>None</li>` : ""}
          </ul>
        </div>
      </div>
    </div>
  `
}

function renderSuggestion(suggestion) {
  const currentVsSuggested = suggestion.currentVsSuggested
  const feedbackState = feedbackStateFor(suggestion.dimensionId)
  return html`
    <div class="ftmCard">
      <div class="ftmCardHeader">
        <div>
          <h4>${suggestion.bucket.replace(/_/g, " ")}</h4>
          <p class="longManeuverMuted">
            ${suggestion.evidence?.speedBand || "mixed"} |
            ${suggestion.evidence?.directionBias || "center"} |
            ${safeCount(suggestion.evidence?.eventCount)} event(s)
          </p>
        </div>
        <div class="ftmFeedbackButtons">
          <button
            class="longManeuverButton ${feedbackState === "accepted" ? "selected" : ""}"
            @click="${() => setDimensionFeedback(suggestion.dimensionId, feedbackState === "accepted" ? "unset" : "accepted")}">
            Matches Experience
          </button>
          <button
            class="longManeuverButton ${feedbackState === "ignored" ? "danger selected" : "danger"}"
            @click="${() => setDimensionFeedback(suggestion.dimensionId, feedbackState === "ignored" ? "unset" : "ignored")}">
            Ignore
          </button>
        </div>
      </div>

      <p><strong>Observed behavior:</strong> ${suggestion.observedBehavior}</p>
      <p><strong>Likely interpretation:</strong> ${suggestion.likelyInterpretation}</p>
      <p><strong>Primary adjustment:</strong> ${suggestion.primaryAdjustment}</p>
      <p><strong>What not to touch yet:</strong> ${suggestion.whatNotToTouchYet}</p>
      <p><strong>If that was wrong, next thing to try:</strong> ${suggestion.ifThatWasWrong}</p>
      <p><strong>Strongest segments:</strong> ${(suggestion.evidence?.segments || []).map((segment) => segment.label).join(", ") || "none"}</p>

      ${currentVsSuggested
        ? html`
          <div class="ftmDeltaBox">
            <strong>Current vs suggested:</strong>
            ${currentVsSuggested.type === "friction_curve"
              ? html`
                <p><code>${currentVsSuggested.family}</code> current: ${renderCurve(currentVsSuggested.current)}</p>
                <p><code>${currentVsSuggested.family}</code> suggested: ${renderCurve(currentVsSuggested.suggested)}</p>
              `
              : html`
                <p>
                  <code>${currentVsSuggested.paramKey || currentVsSuggested.symbol}</code>:
                  ${Number(currentVsSuggested.current).toFixed(3)} -> ${Number(currentVsSuggested.suggested).toFixed(3)}
                </p>
              `}
          </div>
        `
        : html`<p class="longManeuverMuted">No trial adjustment suggested for this dimension.</p>`}

      ${suggestion.plotSvg
        ? html`<div class="ftmPlotWrap"><p class="longManeuverMuted">Saved report includes an inline event plot for this finding.</p></div>`
        : ""}
    </div>
  `
}

function renderPathSummary(path) {
  const selected = path.key === (state.report?.selectedPathKey || state.report?.primaryPathKey)
  return html`
    <div class="ftmCard">
      <div class="ftmCardHeader">
        <div>
          <h4>${path.title}</h4>
          <p class="longManeuverMuted">
            ${path.isPrimary ? "Analyzer recommended" : "Alternate path"}${selected ? " / Active" : ""}
          </p>
        </div>
        <button
          class="longManeuverButton"
          disabled="${() => state.runningAction || selected}"
          @click="${() => selectPath(path.key)}">
          ${selected ? "Active Path" : `Use ${path.title}`}
        </button>
      </div>
      <p>${path.description || ""}</p>
      <p><strong>Why this path:</strong> ${path.whySelected || "No path note available."}</p>
      <p><strong>When to use it:</strong> ${path.whenToUse || "Use the path that best matches the spread of the problem."}</p>
    </div>
  `
}

export function Tuning() {
  initialize()

  return html`
    <div class="longManeuverPage">
      <h2>Lateral Tuning</h2>

      <div class="longManeuverCard">
        <p class="longManeuverIntro">
          Analyze one or more local routes, review deterministic lateral findings, apply a bounded trial, drive, then revert or refine.
        </p>

        <div class="longManeuverActions">
          <button
            class="longManeuverButton"
            disabled="${() => state.runningAction || state.selectedRoutes.length === 0 || !!state.status?.isOnroad}"
            @click="${runAnalyze}">
            Analyze Selected Routes
          </button>
          <button
            class="longManeuverButton danger"
            disabled="${() => state.runningAction || !state.status?.running}"
            @click="${stopAnalyze}">
            Stop Analysis
          </button>
          <button
            class="longManeuverButton"
            disabled="${() => state.runningAction || !state.workspace?.activeTrial}"
            @click="${revertProfile}">
            Revert Trial
          </button>
          <button
            class="longManeuverButton"
            disabled="${() => state.runningAction}"
            @click="${() => {
              state.routes = []
              state.routeProgress = 0
              state.routeTotal = 0
              state.truncatedRoutes = false
              resetRouteStreamState()
              fetchRoutes()
              fetchWorkspace()
              fetchStatus()
            }}">
            Refresh
          </button>
        </div>

        ${() => state.error ? html`<p class="longManeuverError">${state.error}</p>` : ""}

        <div class="longManeuverStatusGrid">
          <p><strong>Status:</strong> ${() => state.status?.state || "idle"}</p>
          <p><strong>Running:</strong> ${() => state.status?.running ? "Yes" : "No"}</p>
          <p><strong>Onroad:</strong> ${() => state.status?.isOnroad ? "Yes" : "No"}</p>
          <p><strong>Updated:</strong> ${() => formatStatusAge(state.status?.updatedAt)}</p>
          <p><strong>Selected Routes:</strong> ${() => state.selectedRoutes.length}</p>
          <p><strong>Progress:</strong> ${() => `${safeCount(state.status?.progress)}/${safeCount(state.status?.total)}`}</p>
          <p><strong>Active Trial:</strong> ${() => state.workspace?.activeTrial?.profileId || "None"}</p>
        </div>

        ${() => state.status?.isOnroad ? html`
          <p class="longManeuverError">FTM analysis is offroad-only. Stop the car and go offroad before starting a run.</p>
        ` : ""}

        ${() => state.status?.currentSegment ? html`
          <div class="longManeuverCurrent">
            <p><strong>Current Segment:</strong> ${state.status.currentSegment}</p>
          </div>
        ` : ""}

        <div class="ftmTwoColumn">
          <section class="ftmCard">
            <div class="ftmCardHeader">
              <div>
                <h3>Local Routes</h3>
                <p class="longManeuverMuted">
                  Pick up to 8 routes from the device. The analyzer prefers rlogs and falls back to qlogs when needed.
                </p>
              </div>
              <button class="longManeuverButton" @click="${clearSelections}">Clear</button>
            </div>

            ${() => state.loadingRoutes ? html`<p class="longManeuverMuted">Loading local routes...</p>` : ""}
            ${() => state.routeTotal ? html`<p class="longManeuverMuted">Route index: ${state.routeProgress}/${state.routeTotal}</p>` : ""}
            ${() => state.truncatedRoutes ? html`<p class="longManeuverMuted">Showing the first ${MAX_RENDERED_ROUTES} routes only.</p>` : ""}

            <div class="ftmRouteList">
              ${() => sortedRoutes().map((route) => html`
                <label class="ftmRouteItem">
                  <input
                    type="checkbox"
                    checked="${() => state.selectedRoutes.includes(route.name)}"
                    @change="${() => toggleRouteSelection(route.name)}" />
                  <span>
                    <strong>${route.timestampLabel}</strong>
                    <small>${route.name}</small>
                  </span>
                </label>
              `)}
            </div>
          </section>

          <section class="ftmCard">
            <div class="ftmCardHeader">
              <div>
                <h3>Workspace</h3>
              </div>
              <button
                class="longManeuverButton danger"
                disabled="${() => state.runningAction || !(state.workspace?.reports || []).length}"
                @click="${clearWorkspace}">
                Clear Workspace
              </button>
            </div>
            ${() => state.loadingWorkspace ? html`<p class="longManeuverMuted">Loading workspace...</p>` : ""}
            <p class="longManeuverMuted">
              Recent reports stay on-device under <code>/data/galaxy/ftm</code>. Loading a report refreshes the suggestion and trial view below.
            </p>
            <div class="ftmWorkspaceList">
              ${() => (state.workspace?.reports || []).length
                ? state.workspace.reports.map((report) => html`
                  <div class="ftmWorkspaceRow">
                    <button class="ftmWorkspaceItem" @click="${() => loadReport(report.reportId)}">
                      <strong>${report.carFingerprint || "Unknown car"}</strong>
                      <span>${(report.routeNames || []).join(", ")}</span>
                      <small>${formatTimestamp(report.createdAt ? new Date(report.createdAt * 1000).toISOString() : "")}</small>
                    </button>
                    <button
                      class="longManeuverButton danger ftmWorkspaceDelete"
                      disabled="${() => state.runningAction}"
                      @click="${() => deleteReport(report.reportId)}">
                      Delete
                    </button>
                  </div>
                `)
                : html`<p class="longManeuverMuted">No tuning reports yet.</p>`}
            </div>
          </section>
        </div>

        ${() => state.report ? html`
          <section class="ftmCard">
            <div class="ftmCardHeader">
              <div>
                <h3>Report Summary</h3>
              </div>
              <button
                class="longManeuverButton danger"
                disabled="${() => state.runningAction || !state.report?.reportId}"
                @click="${() => deleteReport(state.report.reportId)}">
                Delete Report
              </button>
            </div>
            <div class="longManeuverStatusGrid">
              <p><strong>Car:</strong> ${state.report.car?.carFingerprint || "Unknown"}</p>
              <p><strong>Control Path:</strong> ${state.report.car?.controlPath || "unknown"}</p>
              <p><strong>Friction Family:</strong> ${state.report.capabilities?.frictionFamily || "standard"}</p>
              <p><strong>Analyzer Recommended:</strong> ${reportPaths().find((path) => path.isPrimary)?.title || "Recommendations"}</p>
              <p><strong>Active Path:</strong> ${primaryPath()?.title || "Recommendations"}</p>
              <p><strong>Path Choice:</strong> ${state.report.pathSelectionSource === "manual" ? "Manual override" : "Automatic"}</p>
              <p><strong>Nonlinear Torque Map:</strong> ${state.report.capabilities?.nonlinearTorqueMap?.asymmetric ? "Asymmetric left/right siglin" : (state.report.capabilities?.nonlinearTorqueMap ? "Symmetric siglin" : "Not detected")}</p>
              <p><strong>Live Learner Refits Map:</strong> ${state.report.capabilities?.nonlinearTorqueMap ? "No" : "Not applicable"}</p>
              <p><strong>Processed Segments:</strong> ${safeCount(state.report.summary?.processedSegments)}</p>
              <p><strong>qlog Fallback:</strong> ${state.report.summary?.usedQlogFallback ? "Yes" : "No"}</p>
              <p><strong>Samples:</strong> ${safeCount(state.report.summary?.sampleCount)}</p>
            </div>

            <div class="ftmFindings">
              ${reportPaths().map((path) => renderPathSummary(path))}
            </div>

            ${() => (state.report.warnings || []).length ? html`
              <div class="ftmCardSubsection">
                <h4>Warnings</h4>
                <ul>
                  ${(state.report.warnings || []).map((warning) => html`<li>${warning}</li>`)}
                </ul>
              </div>
            ` : ""}

            ${() => (state.report.addTheseParametersAndStartHere || []).length ? html`
              <div class="ftmCardSubsection">
                <h4>Add These Parameters And Start Here</h4>
                <ul>
                  ${(state.report.addTheseParametersAndStartHere || []).map((line) => html`<li>${line}</li>`)}
                </ul>
              </div>
            ` : ""}
          </section>

          <section class="ftmCard">
            <div class="ftmCardHeader">
              <div>
                <h3>Active Findings: ${primaryPath()?.title || "Recommendations"}</h3>
                <p class="longManeuverMuted">
                  ${primaryPath()?.whySelected || "Mark the dimensions that match what the driver felt."}
                </p>
              </div>
              <button
                class="longManeuverButton"
                disabled="${() => state.runningAction || !state.report}"
                @click="${saveFeedback}">
                Save Feedback
              </button>
            </div>

            <textarea
              class="ftmNotes"
              placeholder="Optional tuning notes"
              @input="${(event) => { state.feedbackNotes = event.target.value }}">${() => state.feedbackNotes}</textarea>

            <div class="ftmFindings">
              ${((primaryPath()?.suggestions) || []).map((suggestion) => renderSuggestion(suggestion))}
            </div>
          </section>

          <section class="ftmCard">
            <h3>Trial Profiles</h3>
            <p class="longManeuverMuted">
              Apply one bounded profile at a time. Revert restores the exact advanced-lateral and FTM state that existed before the trial.
            </p>
            <div class="ftmFindings">
              ${reportPaths().length
                ? reportPaths().map((path) => html`
                  <div>
                    <h4>${path.title} Profiles</h4>
                    <p class="longManeuverMuted">${path.whenToUse || ""}</p>
                    ${(path.profiles || []).length
                      ? (path.profiles || []).map((profile) => renderProfile(profile))
                      : html`<p class="longManeuverMuted">No trial profiles generated for this path.</p>`}
                  </div>
                `)
                : html`<p class="longManeuverMuted">No trial profiles generated for this report.</p>`}
            </div>
          </section>
        ` : html`
          <section class="ftmCard">
            <h3>No Active Report</h3>
            <p class="longManeuverMuted">
              Select local routes, run analysis, or open one of the saved reports from the workspace panel.
            </p>
          </section>
        `}
      </div>
    </div>
  `
}
