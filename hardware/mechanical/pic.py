"""Flat-shaded pictures of CAD solids with matplotlib (no Blender needed): assembly steps, overview, part views."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

COL = {"new": "#F08C2E", "old": "#E4E8EE", "servo": "#2A3D5E", "horn": "#F4F4F4", "board": "#3D8B5F",
       "batt": "#505763", "so": "#C9CED6", "leg": "#E9EDF2", "faded": "#F3F5F8"}
_cache = {}


def mesh(shape, key=None):
    k = key if key is not None else id(shape)
    if k in _cache:
        return _cache[k]
    v, t = shape.tessellate(0.25, 0.35)
    V = np.array([[p.x, p.y, p.z] for p in v])
    T = np.array(t, int) if len(t) else np.zeros((0, 3), int)
    _cache[k] = (V, T)
    return V, T


def basis(az=35.0, el=25.0):
    a, e = np.radians(az), np.radians(el)
    d = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])   # from target toward the eye
    r = np.cross([0, 0, 1], d)
    r /= np.linalg.norm(r)
    u = np.cross(d, r)
    return d, r, u


def render(items, path, az=35.0, el=25.0, marks=(), title=None, size=(9, 9), lim=None, labels=(), focus=None):
    """items: [(solid, colour, alpha)]. marks: [(xyz, text)] drawn as red callouts. labels: [(xyz, text)] grey tags."""
    d, r, u = basis(az, el)
    light = d * 0.6 + u * 0.6 + r * 0.3
    light /= np.linalg.norm(light)
    polys, cols, depth = [], [], []
    for s, c, al in items:
        V, T = mesh(s)
        if not len(T):
            continue
        P = V[T]
        n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
        nn = np.linalg.norm(n, axis=1)
        ok = nn > 1e-9
        P, n = P[ok], n[ok] / nn[ok][:, None]
        lam = 0.45 + 0.55 * np.abs(n @ light)
        rgb = np.array(matplotlib.colors.to_rgb(c))
        cc = np.clip(rgb[None, :] * lam[:, None] + 0.02, 0, 1)
        P2 = np.stack([P @ r, P @ u], axis=-1)
        polys.append(P2)
        cols.append(np.concatenate([cc, np.full((len(cc), 1), al)], axis=1))
        depth.append((P @ d).mean(axis=1))
    P2 = np.concatenate(polys)
    C = np.concatenate(cols)
    D = np.concatenate(depth)
    o = np.argsort(D)
    fig, ax = plt.subplots(figsize=size, dpi=110)
    ax.add_collection(PolyCollection(P2[o], facecolors=C[o], edgecolors=C[o], linewidths=0.15))
    allp = P2.reshape(-1, 2)
    lo, hi = allp.min(0), allp.max(0)
    if lim is not None:
        lo, hi = lim
    if focus:
        F = np.concatenate([mesh(f)[0] for f in focus])
        F2 = np.stack([F @ r, F @ u], axis=-1)
        lo, hi = F2.min(0), F2.max(0)
        c, half = (lo + hi) / 2, (hi - lo).max() / 2 * 1.35
        lo, hi = c - half, c + half
    pad = 0.06 * (hi - lo).max()
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    for p, txt in marks:
        p = np.asarray(p, float)
        q = np.array([p @ r, p @ u])
        ax.plot(*q, "o", ms=5, mfc="#E03131", mec="white", mew=0.8, zorder=5)
        if txt:
            ax.annotate(txt, q, xytext=(10, 8), textcoords="offset points", fontsize=8, color="#C92A2A",
                        weight="bold", zorder=6,
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#E03131", lw=0.6))
    for p, txt in labels:
        p = np.asarray(p, float)
        q = np.array([p @ r, p @ u])
        ax.annotate(txt, q, xytext=(14, -14), textcoords="offset points", fontsize=8, color="#1C2B44", zorder=6,
                    arrowprops=dict(arrowstyle="-", color="#5C6B82", lw=0.6),
                    bbox=dict(boxstyle="round,pad=0.25", fc="#F7F9FC", ec="#9AA7BA", lw=0.6))
    ax.set_aspect("equal")
    ax.axis("off")
    if title:
        ax.set_title(title, fontsize=11, color="#1C2B44")
    fig.tight_layout()
    fig.savefig(path, facecolor="white")
    plt.close(fig)
    return lo, hi
