import type { TaskRecord } from "@/lib/types"

type TaskLogPanelProps = {
  task: TaskRecord | null
  logTail: string
  onStop: () => void
}

export function TaskLogPanel({ task, logTail, onStop }: TaskLogPanelProps) {
  return (
    <section className="grid gap-4 xl:grid-cols-[minmax(0,0.8fr)_minmax(340px,1.2fr)]">
      <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
        <div className="flex items-start justify-between gap-3">
          <div>
            <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">当前任务</p>
            <h3 className="mt-2 text-xl font-semibold text-white">{task?.label ?? "空闲"}</h3>
            <p className="mt-2 text-sm leading-6 text-slate-300">
              {task?.running ? `运行中 / PID ${task.pid}` : "当前没有运行任务"}
            </p>
          </div>
          <button
            type="button"
            onClick={onStop}
            className="rounded-full border border-white/10 bg-white/4 px-4 py-2 text-sm text-slate-200 transition hover:bg-white/10"
          >
            停止任务
          </button>
        </div>

        <div className="mt-4 space-y-3">
          <div className="rounded-3xl border border-white/8 bg-black/12 px-4 py-3">
            <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">日志路径</p>
            <p className="mt-2 break-all font-mono text-sm text-slate-200">{task?.log_path ?? "未产生日志"}</p>
          </div>
          <div className="rounded-3xl border border-white/8 bg-black/12 px-4 py-3">
            <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">开始时间</p>
            <p className="mt-2 font-mono text-sm text-slate-200">{task?.started_at ?? "-"}</p>
          </div>
        </div>
      </div>

      <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
        <div className="flex items-center justify-between gap-3">
          <div>
            <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">任务日志</p>
            <p className="mt-2 text-sm text-slate-300">保留最近 200 行，便于快速判断是否可进入下一步。</p>
          </div>
        </div>
        <pre className="mt-4 max-h-[420px] overflow-auto rounded-[24px] border border-white/8 bg-slate-950/70 p-4 font-mono text-xs leading-6 text-slate-200">
          {logTail || "等待状态数据..."}
        </pre>
      </div>
    </section>
  )
}
