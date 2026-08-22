import type { VisionMode } from "@/lib/types"

type VisionPanelProps = {
  leftSrc: string
  rightSrc: string
  topics: {
    left_image: string
    right_image: string
  }
  visionMode: VisionMode
  onModeChange: (mode: "snapshot" | "stream") => void
}

export function VisionPanel({ leftSrc, rightSrc, topics, visionMode, onModeChange }: VisionPanelProps) {
  return (
    <section className="grid gap-4">
      <div className="flex flex-col gap-3 md:flex-row md:items-center md:justify-between">
        <div>
          <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">双目视觉反馈</p>
          <p className="mt-2 text-sm leading-6 text-slate-300">
            左：{topics.left_image} / 右：{topics.right_image}
          </p>
        </div>
        <div className="flex items-center gap-2 rounded-full border border-white/10 bg-white/4 p-1">
          <button
            type="button"
            onClick={() => onModeChange("snapshot")}
            className={`rounded-full px-4 py-2 text-sm ${
              visionMode.mode === "snapshot" ? "bg-white text-slate-950" : "text-slate-300"
            }`}
          >
            快照
          </button>
          <button
            type="button"
            onClick={() => onModeChange("stream")}
            className={`rounded-full px-4 py-2 text-sm ${
              visionMode.mode === "stream" ? "bg-white text-slate-950" : "text-slate-300"
            }`}
          >
            流式预留
          </button>
        </div>
      </div>

      <div className="grid gap-4 xl:grid-cols-2">
        {[
          { title: "左相机", src: leftSrc },
          { title: "右相机", src: rightSrc },
        ].map((item) => (
          <figure
            key={item.title}
            className="overflow-hidden rounded-[28px] border border-white/10 bg-black/25 shadow-xl shadow-black/20"
          >
            <figcaption className="flex items-center justify-between border-b border-white/8 px-4 py-3 text-sm text-slate-300">
              <span>{item.title}</span>
              <span className="text-xs tracking-[0.18em] text-slate-500 uppercase">{visionMode.mode}</span>
            </figcaption>
            <img
              src={item.src}
              alt={item.title}
              className="aspect-video w-full object-cover"
            />
          </figure>
        ))}
      </div>
    </section>
  )
}
