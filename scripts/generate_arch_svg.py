#!/usr/bin/env python3
"""Generate 5-Layer Platform Architecture Diagram (SVG & PNG)."""

import os
import subprocess


def build_svg():
    width = 1600
    height = 980

    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
               f'width="{width}" height="{height}">')

    # Definitions: Gradients, Filters, Shadows
    svg.append('<defs>')

    # Background gradient
    svg.append('''
        <linearGradient id="bgGradient" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#050b14"/>
            <stop offset="50%" stop-color="#0a1424"/>
            <stop offset="100%" stop-color="#050a12"/>
        </linearGradient>
    ''')

    # Ambient glow filters
    svg.append('''
        <filter id="glowPink" x="-20%" y="-20%" width="140%" height="140%">
            <feGaussianBlur stdDeviation="8" result="blur"/>
            <feComposite in="SourceGraphic" in2="blur" operator="over"/>
        </filter>
        <filter id="plateShadow" x="-20%" y="-20%" width="140%" height="140%">
            <feDropShadow dx="0" dy="12" stdDeviation="10" flood-color="#000000" flood-opacity="0.45"/>
        </filter>
    ''')

    # Layer Color Gradients (Top, Left, Right faces)
    # Layer 5: Apps (Pink/Rose)
    svg.append('''
        <linearGradient id="l5Top" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#fb7185"/>
            <stop offset="100%" stop-color="#e11d48"/>
        </linearGradient>
        <linearGradient id="l5Left" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#be123c"/>
            <stop offset="100%" stop-color="#881337"/>
        </linearGradient>
        <linearGradient id="l5Right" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#9f1239"/>
            <stop offset="100%" stop-color="#4c0519"/>
        </linearGradient>
    ''')

    # Layer 4: Middleware & Framework (Purple/Violet)
    svg.append('''
        <linearGradient id="l4Top" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#c084fc"/>
            <stop offset="100%" stop-color="#9333ea"/>
        </linearGradient>
        <linearGradient id="l4Left" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#7e22ce"/>
            <stop offset="100%" stop-color="#581c87"/>
        </linearGradient>
        <linearGradient id="l4Right" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#6b21a8"/>
            <stop offset="100%" stop-color="#3b0764"/>
        </linearGradient>
    ''')

    # Layer 3: Native Core & Digital Twin (Deep Blue / Sky Blue)
    svg.append('''
        <linearGradient id="l3Top" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#38bdf8"/>
            <stop offset="100%" stop-color="#0284c7"/>
        </linearGradient>
        <linearGradient id="l3Left" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#0369a1"/>
            <stop offset="100%" stop-color="#075985"/>
        </linearGradient>
        <linearGradient id="l3Right" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#0c4a6e"/>
            <stop offset="100%" stop-color="#082f49"/>
        </linearGradient>
    ''')

    # Sub-plate on Layer 3: Simulation & ML Runtime (Amber/Orange accent)
    svg.append('''
        <linearGradient id="l3SubTop" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#fb923c"/>
            <stop offset="100%" stop-color="#ea580c"/>
        </linearGradient>
        <linearGradient id="l3SubLeft" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#c2410c"/>
            <stop offset="100%" stop-color="#9a3412"/>
        </linearGradient>
        <linearGradient id="l3SubRight" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#9a3412"/>
            <stop offset="100%" stop-color="#7c2d12"/>
        </linearGradient>
    ''')

    # Layer 2: Real-Time HAL & MCU (Cyan/Teal)
    svg.append('''
        <linearGradient id="l2Top" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#2dd4bf"/>
            <stop offset="100%" stop-color="#0d9488"/>
        </linearGradient>
        <linearGradient id="l2Left" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#0f766e"/>
            <stop offset="100%" stop-color="#115e59"/>
        </linearGradient>
        <linearGradient id="l2Right" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#115e59"/>
            <stop offset="100%" stop-color="#042f2e"/>
        </linearGradient>
    ''')

    # Layer 1: Hardware & Mechatronics (Emerald / Bright Green)
    svg.append('''
        <linearGradient id="l1Top" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#4ade80"/>
            <stop offset="100%" stop-color="#16a34a"/>
        </linearGradient>
        <linearGradient id="l1Left" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#15803d"/>
            <stop offset="100%" stop-color="#166534"/>
        </linearGradient>
        <linearGradient id="l1Right" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stop-color="#166534"/>
            <stop offset="100%" stop-color="#052e16"/>
        </linearGradient>
    ''')

    svg.append('</defs>')

    # Background canvas
    svg.append(f'<rect width="{width}" height="{height}" fill="url(#bgGradient)"/>')

    # Atmospheric blur spots
    svg.append('''
        <g opacity="0.18">
            <circle cx="280" cy="220" r="280" fill="#f43f5e" filter="url(#glowPink)"/>
            <circle cx="450" cy="500" r="320" fill="#0284c7" filter="url(#glowPink)"/>
            <circle cx="300" cy="800" r="300" fill="#10b981" filter="url(#glowPink)"/>
            <circle cx="1200" cy="480" r="380" fill="#6366f1" filter="url(#glowPink)"/>
        </g>
    ''')

    # Grid lines
    svg.append('''
        <g opacity="0.04" stroke="#ffffff" stroke-width="1">
            <line x1="0" y1="160" x2="1600" y2="160"/>
            <line x1="0" y1="320" x2="1600" y2="320"/>
            <line x1="0" y1="480" x2="1600" y2="480"/>
            <line x1="0" y1="640" x2="1600" y2="640"/>
            <line x1="0" y1="800" x2="1600" y2="800"/>
        </g>
    ''')

    # Header
    svg.append('''
        <g id="header" text-anchor="middle">
            <text x="800" y="80"
                  font-family="-apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif"
                  font-size="44" font-weight="800" fill="#ffffff" letter-spacing="-0.5">
                Synapse Platform Architecture
            </text>
            <text x="800" y="115"
                  font-family="-apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif"
                  font-size="16" font-weight="500" fill="#94a3b8" letter-spacing="0.5">
                MODULAR 5-LAYER EMBODIED AI &amp; DETERMINISTIC REAL-TIME ROBOTIC STACK
            </text>
        </g>
    ''')

    # Isometric Slab Function
    def render_slab(cx, cy, rx, ry, h, top_id, left_id, right_id, border_color="#ffffff", border_op=0.25):
        t = (cx, cy - ry)
        r = (cx + rx, cy)
        b = (cx, cy + ry)
        left_pt = (cx - rx, cy)

        b_ext = (cx, cy + ry + h)
        l_ext = (cx - rx, cy + h)
        r_ext = (cx + rx, cy + h)

        res = '<g filter="url(#plateShadow)">'
        res += (f'<polygon points="{left_pt[0]},{left_pt[1]} {b[0]},{b[1]} {b_ext[0]},{b_ext[1]} '
                f'{l_ext[0]},{l_ext[1]}" fill="url(#{left_id})" stroke="{border_color}" '
                f'stroke-opacity="{border_op * 0.5}" stroke-width="0.8"/>')
        res += (f'<polygon points="{b[0]},{b[1]} {r[0]},{r[1]} {r_ext[0]},{r_ext[1]} {b_ext[0]},{b_ext[1]}" '
                f'fill="url(#{right_id})" stroke="{border_color}" '
                f'stroke-opacity="{border_op * 0.5}" stroke-width="0.8"/>')
        res += (f'<polygon points="{t[0]},{t[1]} {r[0]},{r[1]} {b[0]},{b[1]} {left_pt[0]},{left_pt[1]}" '
                f'fill="url(#{top_id})" stroke="{border_color}" '
                f'stroke-opacity="{border_op}" stroke-width="1.2"/>')
        res += (f'<polyline points="{left_pt[0]},{left_pt[1]} {t[0]},{t[1]} {r[0]},{r[1]}" '
                f'fill="none" stroke="#ffffff" stroke-opacity="0.4" stroke-width="1.5"/>')
        res += '</g>'
        return res

    # Function to draw connection socket cube
    def render_socket_cube(cx, cy, size=11, height=12, top_color="#ffffff",
                           left_color="#94a3b8", right_color="#64748b"):
        rx = size * 1.0
        ry = size * 0.5
        t = (cx, cy - ry)
        r = (cx + rx, cy)
        b = (cx, cy + ry)
        left_pt = (cx - rx, cy)

        b_ext = (cx, cy + ry + height)
        l_ext = (cx - rx, cy + height)
        r_ext = (cx + rx, cy + height)

        res = '<g>'
        res += (f'<polygon points="{left_pt[0]},{left_pt[1]} {b[0]},{b[1]} {b_ext[0]},{b_ext[1]} '
                f'{l_ext[0]},{l_ext[1]}" fill="{left_color}" stroke="#000000" stroke-opacity="0.3" '
                f'stroke-width="0.5"/>')
        res += (f'<polygon points="{b[0]},{b[1]} {r[0]},{r[1]} {r_ext[0]},{r_ext[1]} {b_ext[0]},{b_ext[1]}" '
                f'fill="{right_color}" stroke="#000000" stroke-opacity="0.3" stroke-width="0.5"/>')
        res += (f'<polygon points="{t[0]},{t[1]} {r[0]},{r[1]} {b[0]},{b[1]} {left_pt[0]},{left_pt[1]}" '
                f'fill="{top_color}" stroke="#ffffff" stroke-opacity="0.6" stroke-width="0.8"/>')
        res += '</g>'
        return res

    cx = 380
    rx = 270
    ry = 115
    slab_h = 24

    layers_data = [
        {
            "id": "L1",
            "y": 780,
            "top": "l1Top", "left": "l1Left", "right": "l1Right",
            "accent": "#4ade80", "dim_accent": "#16a34a",
            "title": "Hardware &amp; Mechatronics",
            "layer_tag": "LAYER 1",
            "desc": "Physical robot body, low-impedance actuators, power conditioning &amp; analog sensors",
            "badges": ["20-DoF Joint Actuators", "MPU6050 6-DoF IMU", "4x Foot FSR Sensors",
                       "2S LiPo / Dual Buck Regulators"]
        },
        {
            "id": "L2",
            "y": 640,
            "top": "l2Top", "left": "l2Left", "right": "l2Right",
            "accent": "#2dd4bf", "dim_accent": "#0d9488",
            "title": "Hardware Abstraction Layer (HAL)",
            "layer_tag": "LAYER 2",
            "desc": "Deterministic microcontroller firmware, real-time kinematics &amp; fail-safe interlocks",
            "badges": ["STM32F411 BlackPill", "FreeRTOS 200 Hz", "S-Curve Trajectory Gen",
                       "Safety Watchdog Interlocks"]
        },
        {
            "id": "L3",
            "y": 500,
            "top": "l3Top", "left": "l3Left", "right": "l3Right",
            "accent": "#38bdf8", "dim_accent": "#0284c7",
            "title": "Native Core &amp; Digital Twin",
            "layer_tag": "LAYER 3",
            "desc": "Sub-millisecond physics simulation, locked-memory IPC &amp; neural inference engine",
            "badges": ["MuJoCo 3.8+ (Metal MPS)", "Lock-Free SPSC Queues", "ONNX Runtime / C++",
                       "COBS + CRC-16 Protocol"]
        },
        {
            "id": "L4",
            "y": 360,
            "top": "l4Top", "left": "l4Left", "right": "l4Right",
            "accent": "#c084fc", "dim_accent": "#9333ea",
            "title": "Robotics Framework &amp; Middleware",
            "layer_tag": "LAYER 4",
            "desc": "Orchestration backbone, kinematic state publishing, coordinate frames &amp; telemetry",
            "badges": ["ROS 2 Jazzy Core", "robot_state_publisher", "TF2 Dynamic Transforms", "RViz2 Real-Time Dash"]
        },
        {
            "id": "L5",
            "y": 220,
            "top": "l5Top", "left": "l5Left", "right": "l5Right",
            "accent": "#fb7185", "dim_accent": "#e11d48",
            "title": "Applications &amp; Cognition",
            "layer_tag": "LAYER 5",
            "desc": "High-level goal conditioning, neural policy locomotion &amp; behavioral state machines",
            "badges": ["RL Policy (Titan / ResNet)", "Gait &amp; Behavior FSM", "Vision &amp; Depth Perception",
                       "Autonomous Locomotion Loop"]
        }
    ]

    # Draw slabs from bottom (L1) to top (L5)
    for ldata in layers_data:
        svg.append(f'<!-- {ldata["layer_tag"]} SLAB -->')
        svg.append(render_slab(cx, ldata["y"], rx, ry, slab_h, ldata["top"], ldata["left"], ldata["right"],
                               border_color=ldata["accent"]))

        if ldata["id"] == "L3":
            sub_cx = cx + 30
            sub_cy = ldata["y"] + 15
            sub_rx = 115
            sub_ry = 50
            sub_h = 16
            svg.append('<!-- L3 Sub-plate (Runtime & Physics Engine) -->')
            svg.append(render_slab(sub_cx, sub_cy, sub_rx, sub_ry, sub_h, "l3SubTop", "l3SubLeft", "l3SubRight",
                                   border_color="#fb923c"))
            svg.append(f'''
                <g transform="translate({sub_cx}, {sub_cy - 4})">
                    <text x="0" y="0" text-anchor="middle"
                          font-family="-apple-system, BlinkMacSystemFont, sans-serif"
                          font-size="11" font-weight="700" fill="#ffffff" letter-spacing="0.5">
                        PHYSICS &amp; ML RUNTIME
                    </text>
                </g>
            ''')

    # Draw connection sockets and pointer lines to the right
    label_start_x = 760

    for ldata in layers_data:
        ly = ldata["y"]
        sock_x = cx + rx * 0.58
        sock_y = ly + ry * 0.42

        svg.append(render_socket_cube(sock_x, sock_y - 12, size=11, height=12, top_color=ldata["accent"],
                                      left_color=ldata["dim_accent"], right_color="#1e293b"))

        pin_start_x = sock_x
        pin_start_y = sock_y - 12
        line_target_y = ly + 4

        mid_x = label_start_x - 30
        path_d = f"M {pin_start_x} {pin_start_y} L {mid_x} {line_target_y} L {label_start_x} {line_target_y}"

        svg.append(f'''
            <g id="pointer_{ldata['id']}">
                <path d="{path_d}" fill="none" stroke="{ldata['accent']}" stroke-width="2"
                      stroke-linecap="round" stroke-linejoin="round" opacity="0.85"/>
                <circle cx="{label_start_x}" cy="{line_target_y}" r="4" fill="{ldata['accent']}"/>
                <circle cx="{label_start_x}" cy="{line_target_y}" r="7" fill="none"
                        stroke="{ldata['accent']}" stroke-width="1.5" opacity="0.5"/>
            </g>
        ''')

        text_x = label_start_x + 22
        svg.append(f'''
            <g id="label_{ldata['id']}" transform="translate({text_x}, {line_target_y})">
                <rect x="0" y="-36" width="70" height="18" rx="4"
                      fill="{ldata['accent']}" fill-opacity="0.18" stroke="{ldata['accent']}" stroke-width="1"/>
                <text x="35" y="-23" text-anchor="middle"
                      font-family="-apple-system, BlinkMacSystemFont, sans-serif"
                      font-size="10.5" font-weight="800" fill="{ldata['accent']}" letter-spacing="0.5">
                    {ldata['layer_tag']}
                </text>
                <text x="82" y="-23"
                      font-family="-apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif"
                      font-size="24" font-weight="700" fill="#ffffff" letter-spacing="-0.3">
                    {ldata['title']}
                </text>
                <text x="0" y="-2"
                      font-family="-apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif"
                      font-size="13.5" font-weight="400" fill="#cbd5e1">
                    {ldata['desc']}
                </text>
                <g transform="translate(0, 14)">
        ''')

        bx = 0
        for badge in ldata["badges"]:
            b_width = len(badge) * 6.8 + 16
            svg.append(f'''
                <rect x="{bx}" y="0" width="{b_width}" height="20" rx="5"
                      fill="#1e293b" fill-opacity="0.8" stroke="#475569" stroke-width="0.8"/>
                <text x="{bx + b_width/2}" y="13.5" text-anchor="middle"
                      font-family="-apple-system, BlinkMacSystemFont, sans-serif"
                      font-size="10.5" font-weight="600" fill="#e2e8f0">
                    {badge}
                </text>
            ''')
            bx += b_width + 8

        svg.append('''
                </g>
            </g>
        ''')

    # Watermark / Footer
    svg.append(f'''
        <g id="footer" transform="translate(80, {height - 35})">
            <text x="0" y="0" font-family="-apple-system, BlinkMacSystemFont, sans-serif"
                  font-size="12" font-weight="500" fill="#64748b">
                Synapse Humanoid Robotics · Decoupled 5-Tier Platform Architecture
            </text>
        </g>
    ''')

    svg.append('</svg>')
    return "\n".join(svg)


if __name__ == "__main__":
    content = build_svg()
    os.makedirs("docs/images", exist_ok=True)
    out_svg = "docs/images/platform_architecture_5layers.svg"
    out_png = "docs/images/platform_architecture_5layers.png"

    with open(out_svg, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"Generated {out_svg}")

    cmd = [
        "/opt/homebrew/bin/rsvg-convert",
        "-w", "1600",
        "-h", "980",
        out_svg,
        "-o", out_png,
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode == 0:
        print(f"Generated {out_png}")
    else:
        print(f"rsvg-convert error: {res.stderr}")
