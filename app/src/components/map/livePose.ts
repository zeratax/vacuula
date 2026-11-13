// Live robot position: the state behind the overlay on the Map page.
//
// The server (livepose.py) sends poses in the robot's own map frame (metres, x right, y up, theta
// counter-clockwise). `calibration.maps[mapId].robot_to_px` = [[a, b, c], [d, e, f]] turns that into the cell
// coordinates of a persistent map: col = a*x + b*y + c, row = d*x + e*y + f. Maps without an entry get nothing.
//
// The state lives in a small store of its own, not in AppContext: poses arrive up to 5 times a second and must
// only repaint the overlay canvas, not re-render the app.

import { useSyncExternalStore } from 'react'

export type LivePoseStatus = 'none' | 'pose' | 'localizing' | 'reset' | 'unmapped' | 'stale'
// reset: pose is (0,0,0), the robot is docked or lifted. unmapped: this clean does not use the saved map.

export interface RobotPose { x: number, y: number, theta: number, segment: number, clean: number }
export type TrailPoint = [number, number]
type Affine = [[number, number, number], [number, number, number]]

export interface LivePoseCalibration { maps: { [mapId: string]: { robot_to_px: Affine } } }

/** GET /api/robot/<serial>/pose */
export interface LivePoseSnapshot {
    serial: string
    status: LivePoseStatus
    ts: number | null      // server unix seconds of the last status change / pose
    now: number            // server unix seconds when this was sent
    pose: RobotPose | null
    trail: TrailPoint[][]
    calibration: LivePoseCalibration
}

/** `robot_pos` event on the /connect stream */
export type RobotPosEvent = { serial: string, ts: number } & (
    | { kind: 'pose', x: number, y: number, theta: number, segment: number, clean: number }
    | { kind: 'localizing' | 'reset' | 'unmapped' | 'stale' }
)

export interface LivePose {
    status: LivePoseStatus
    pose: RobotPose | null            // last known, kept while reset / stale
    trail: TrailPoint[][]             // robot metres, one polyline per segment, current clean only
    since: number | null              // browser clock (ms) of the last status change, for hiding old poses
    serverTs: number | null           // server clock (s) of the newest thing applied
    calibration: LivePoseCalibration
}

export const NO_LIVE_POSE: LivePose = { status: 'none', pose: null, trail: [], since: null, serverTs: null, calibration: { maps: {} } }

/** A reset (docked) or stale robot disappears from the map after this long. */
export const LIVE_POSE_HIDE_AFTER_MS = 5 * 60 * 1000
/** No pose event for this long while "pose": grey it out, whatever the server says (the server sends one at least every 2 s). */
export const LIVE_POSE_STALE_AFTER_MS = 5000

const TRAIL_STEP_M = 0.03
const TRAIL_MAX_POINTS = 6000

export function livePoseFromSnapshot(snap: LivePoseSnapshot, cur: LivePose = NO_LIVE_POSE): LivePose {
    // Events that arrived while the request was in flight are newer than the snapshot: keep them, take the calibration.
    // A snapshot without data (server restarted) or one made more than a second after our newest event wins.
    const eventsAreNewer = cur.serverTs !== null && snap.ts !== null && snap.ts < cur.serverTs && cur.serverTs >= snap.now - 1
    if (eventsAreNewer) return { ...cur, calibration: snap.calibration }
    return {
        status: snap.status,
        pose: snap.pose,
        trail: snap.trail,
        since: snap.ts === null ? null : Date.now() - (snap.now - snap.ts) * 1000,
        serverTs: snap.ts,
        calibration: snap.calibration,
    }
}

export function applyRobotPos(cur: LivePose, e: RobotPosEvent): LivePose {
    if (e.kind !== 'pose') {
        // localizing / unmapped = a new clean has started, its trail starts empty (like on the server)
        const fresh = e.kind === 'localizing' || e.kind === 'unmapped'
        return { ...cur, status: e.kind, since: Date.now(), serverTs: e.ts, trail: fresh ? [] : cur.trail }
    }
    // a changed `clean` is a new clean even if its first events were never seen
    const sameClean = cur.pose?.clean === e.clean
    const trail = sameClean ? cur.trail.slice() : []
    if (!sameClean || cur.pose?.segment !== e.segment || trail.length === 0) trail.push([])
    const seg = trail[trail.length - 1].slice()
    trail[trail.length - 1] = seg
    const last = seg[seg.length - 1]
    if (!last || Math.hypot(e.x - last[0], e.y - last[1]) >= TRAIL_STEP_M) seg.push([e.x, e.y])
    for (let n = trail.reduce((s, t) => s + t.length, 0); n > TRAIL_MAX_POINTS && trail.length > 0; n--) {
        trail[0] = trail[0].slice(1)
        if (trail[0].length === 0) trail.shift()
    }
    return {
        ...cur, status: 'pose', trail, since: Date.now(), serverTs: e.ts,
        pose: { x: e.x, y: e.y, theta: e.theta, segment: e.segment, clean: e.clean },
    }
}

/** What to draw at `now`: nothing, or the robot (grey when stale, localizing or silent for a while). */
export function livePoseView(live: LivePose, now = Date.now()): { hidden: boolean, grey: boolean } {
    const age = live.since === null ? 0 : now - live.since
    switch (live.status) {
        case 'none':
        case 'unmapped': return { hidden: true, grey: false }
        case 'localizing': return { hidden: false, grey: true }
        case 'reset': return { hidden: age > LIVE_POSE_HIDE_AFTER_MS, grey: false }
        case 'stale': return { hidden: age > LIVE_POSE_HIDE_AFTER_MS, grey: true }
        case 'pose': return { hidden: age > LIVE_POSE_HIDE_AFTER_MS, grey: age > LIVE_POSE_STALE_AFTER_MS }
    }
}

/** Milliseconds until livePoseView changes by itself (nothing new arriving), or null. */
export function nextLivePoseChange(live: LivePose, now = Date.now()): number | null {
    if (live.since === null) return null
    const at = live.status === 'pose' ? [LIVE_POSE_STALE_AFTER_MS, LIVE_POSE_HIDE_AFTER_MS]
        : live.status === 'reset' || live.status === 'stale' ? [LIVE_POSE_HIDE_AFTER_MS] : []
    const left = at.map(ms => live.since! + ms - now).filter(ms => ms > 0)
    return left.length ? Math.min(...left) : null
}

// --- the store (module level, like the loaders in ../../loaders) ---

let current: LivePose = NO_LIVE_POSE
const listeners = new Set<() => void>()

export const livePoseStore = {
    get: () => current,
    update(fn: (cur: LivePose) => LivePose) {
        current = fn(current)
        listeners.forEach(l => l())
    },
    subscribe(listener: () => void) {
        listeners.add(listener)
        return () => { listeners.delete(listener) }
    },
}

export const useLivePose = () => useSyncExternalStore(livePoseStore.subscribe, livePoseStore.get)
