import { ServerCog } from "lucide-react"

import { Badge } from "@/components/ui/badge"
import type { AdminStatus } from "@/lib/types"

type SystemConsolePageProps = {
  status: AdminStatus
}

export function SystemConsolePage({ status }: SystemConsolePageProps) {
  return (
    <section className="space-y-4">
      <div className="rounded-[30px] border border-white/10 bg-card/90 p-6 backdrop-blur-xl">
        <div className="flex flex-col gap-4 xl:flex-row xl:items-end xl:justify-between">
          <div className="space-y-4">
            <Badge variant="outline" className="border-cyan-300/18 bg-cyan-400/10 text-cyan-100">
              Platform / System Console
            </Badge>
            <div>
              <h2 className="text-3xl font-semibold tracking-tight text-white md:text-4xl">系统路径与接口基座</h2>
              <p className="mt-3 max-w-4xl text-sm leading-7 text-slate-300 md:text-base">
                这个版面只负责展示机器人路径、标定工具、视觉话题和可执行接口，避免与标定操作流互相打断。
              </p>
            </div>
          </div>

          <div className="inline-flex items-center gap-3 rounded-full border border-white/10 bg-white/5 px-4 py-3 text-sm text-slate-200">
            <ServerCog className="size-4 text-cyan-200" />
            Python {status.python}
          </div>
        </div>
      </div>

      <section className="grid gap-4 xl:grid-cols-[minmax(0,1.05fr)_minmax(0,0.95fr)]">
        <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
          <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">关键路径</p>
          <div className="mt-4 grid gap-3">
            {[
              { label: "仓库根路径", value: status.paths.repo_root },
              { label: "工作区", value: status.paths.workspace_root },
              { label: "Mercury 工具", value: `${status.paths.mercury_root}/calibrate_stereo.py` },
              { label: "采样目录", value: status.paths.calib_dir },
              { label: "占位标定", value: status.paths.placeholder_calib },
              { label: "仓库标定目录", value: status.paths.repo_calib_dir },
            ].map((item) => (
              <div key={item.label} className="rounded-3xl border border-white/8 bg-white/4 p-4">
                <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">{item.label}</p>
                <p className="mt-2 break-all font-mono text-sm text-slate-100">{item.value}</p>
              </div>
            ))}
          </div>
        </div>

        <div className="space-y-4">
          <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
            <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">双目话题</p>
            <div className="mt-4 grid gap-3">
              <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
                <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">左目</p>
                <p className="mt-2 break-all font-mono text-sm text-slate-100">{status.topics.left_image}</p>
              </div>
              <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
                <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">右目</p>
                <p className="mt-2 break-all font-mono text-sm text-slate-100">{status.topics.right_image}</p>
              </div>
            </div>
          </div>

          <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
            <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">接口与命令</p>
            <div className="mt-4 space-y-3">
              <div className="rounded-3xl border border-white/8 bg-white/4 p-4 text-sm leading-7 text-slate-200">
                `/api/status` / `/api/tasks/start` / `/api/tasks/stop` / `/api/calibration/*` / `/api/vision/*`
              </div>
              {status.commands.map((command) => (
                <div key={command.id} className="rounded-3xl border border-white/8 bg-black/12 p-4">
                  <p className="text-sm font-semibold text-white">{command.label}</p>
                  <p className="mt-1 text-sm text-slate-300">{command.description}</p>
                  <p className="mt-3 break-all font-mono text-xs leading-6 text-slate-400">{command.command}</p>
                </div>
              ))}
            </div>
          </div>
        </div>
      </section>
    </section>
  )
}
