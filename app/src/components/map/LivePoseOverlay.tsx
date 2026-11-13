import { useEffect, useRef, useState } from 'react'
import styles from './map.module.scss'
import { ROBOT_RADIUS_MM } from './drawingFunctions'
import { livePoseView, nextLivePoseChange, useLivePose, type LivePose, type TrailPoint } from './livePose'
import type { Offset } from './types'

const TRAIL_COLOR = 'rgba(74, 179, 245, 0.75)'
const ROBOT_COLOR = '#ff5a1f'
const GREY = 'rgba(150, 150, 150, 0.7)'

function drawLivePose(
    ctx: CanvasRenderingContext2D,
    live: LivePose,
    mapId: string | undefined,
    zoom: number,
    offset: Offset,
) {
    const cal = mapId ? live.calibration.maps[mapId] : undefined
    const { hidden, grey } = livePoseView(live)
    if (!cal || hidden) return
    const [[a, b, c], [d, e, f]] = cal.robot_to_px

    // cell (col, row) is drawn centred at offset + (col + .5, row + .5) * zoom
    const toScreen = (x: number, y: number): TrailPoint => [
        offset.x + (a * x + b * y + c + 0.5) * zoom,
        offset.y + (d * x + e * y + f + 0.5) * zoom,
    ]

    ctx.save()
    ctx.lineJoin = 'round'
    ctx.lineCap = 'round'

    ctx.strokeStyle = grey ? GREY : TRAIL_COLOR
    ctx.lineWidth = Math.max(1.5, zoom * 0.35)
    for (const seg of live.trail) {
        if (seg.length < 2) continue
        ctx.beginPath()
        seg.forEach(([x, y], i) => {
            const [sx, sy] = toScreen(x, y)
            if (i === 0) ctx.moveTo(sx, sy)
            else ctx.lineTo(sx, sy)
        })
        ctx.stroke()
    }

    if (live.pose) {
        const { x, y, theta } = live.pose
        const [sx, sy] = toScreen(x, y)
        // the map's own scale: px per metre from the calibration (24.1 -> 165 mm = 4 px); 25 if it is degenerate
        const r = (ROBOT_RADIUS_MM / 1000) * (Math.sqrt(Math.abs(a * e - b * d)) || 25) * zoom
        // heading through the linear part of the transform (px rows run downwards: (cos, -sin) for the default)
        const dx = a * Math.cos(theta) + b * Math.sin(theta)
        const dy = d * Math.cos(theta) + e * Math.sin(theta)
        const len = Math.hypot(dx, dy) || 1

        ctx.fillStyle = grey ? GREY : ROBOT_COLOR
        ctx.beginPath()
        ctx.arc(sx, sy, r, 0, Math.PI * 2)
        ctx.fill()
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.9)'
        ctx.lineWidth = Math.max(1.5, zoom * 0.25)
        ctx.stroke()
        ctx.beginPath()
        ctx.moveTo(sx, sy)
        ctx.lineTo(sx + (dx / len) * r, sy + (dy / len) * r)
        ctx.stroke()
    }
    ctx.restore()
}

interface LivePoseOverlayProps {
    mapId: string
    zoom: number
    offset: Offset
}

// Trail + robot on their own canvas stacked over the map canvas, so a new pose repaints only this one
// (the map itself is ~160k cells and must not be redrawn 5 times a second). Same zoom / offset as the map.
export default function LivePoseOverlay({ mapId, zoom, offset }: LivePoseOverlayProps) {
    const live = useLivePose()
    const canvasRef = useRef<HTMLCanvasElement>(null)
    const [size, setSize] = useState({ width: 0, height: 0 })
    const [expiredTick, setExpiredTick] = useState(0)

    // follow the container's size, like the map canvas does
    useEffect(() => {
        const parent = canvasRef.current?.parentElement
        if (!parent) return
        const resize = () => {
            const { width, height } = parent.getBoundingClientRect()
            setSize({ width, height })
        }
        resize()
        const ro = new ResizeObserver(resize)
        ro.observe(parent)
        return () => ro.disconnect()
    }, [])

    // grey after 5 s without a pose, hidden after 5 min of reset / stale: repaint when that time is up
    useEffect(() => {
        const left = nextLivePoseChange(live)
        if (left === null) return
        const timer = setTimeout(() => setExpiredTick(t => t + 1), left + 50)
        return () => clearTimeout(timer)
    }, [live, expiredTick])

    useEffect(() => {
        const id = requestAnimationFrame(() => {
            const canvas = canvasRef.current
            const ctx = canvas?.getContext('2d')
            if (!canvas || !ctx) return
            ctx.clearRect(0, 0, canvas.width, canvas.height)
            drawLivePose(ctx, live, mapId, zoom, offset)
        })
        return () => cancelAnimationFrame(id)
    }, [live, mapId, zoom, offset, size, expiredTick])

    return <canvas ref={canvasRef} className={styles.livePose} width={size.width} height={size.height} />
}
