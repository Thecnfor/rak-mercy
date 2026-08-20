import { Activity, Camera, CheckCircle2, Clock3, Workflow } from "lucide-react"

import { Badge } from "@/components/ui/badge"
import { Card, CardContent } from "@/components/ui/card"
import type { AdminStatus, Metrics, TaskRecord, VisionMode } from "@/lib/types"

type OverviewPageProps = {
  status: AdminStatus
  metrics: Metrics
  task: TaskRecord | null
  visionMode: VisionMode
}

export function OverviewPage({ status, metrics, task, visionMode }: OverviewPageProps) {
  const currentStep = status.calibration.wizard.steps.find((step) => step.status === "current")

  return (
    <section className="space-y-4">
      <Card className="overflow-hidden border-white/10 bg-[linear-gradient(135deg,rgba(8,17,30,0.98),rgba(9,30,48,0.92)_40%,rgba(6,15,25,0.96))] shadow-2xl shadow-black/30">
        <CardContent className="space-y-6 p-6 md:p-8">
          <div className="flex flex-col gap-4 xl:flex-row xl:items-end xl:justify-between">
            <div className="space-y-4">
              <Badge className="border-none bg-emerald-400/12 text-emerald-100">Deyes / Console Overview</Badge>
              <div className="space-y-3">
                <h2 className="max-w-5xl text-4xl leading-tight font-semibold tracking-tight text-white md:text-5xl">
                  将标定、视觉与日志拆成多中心页，让 admin_gui 真正像操作台而不是单页堆叠。
                </h2>
                <p className="max-w-4xl text-base leading-7 text-slate-300 md:text-lg">
                  总览页只负责回答当前状态、当前阶段和当前风险，双目标定改由左侧导航进入独立中心页处理。
                </p>
              </div>
            </div>

            <div className="grid gap-3 sm:grid-cols-2">
              <div className="rounded-[26px] border border-white/10 bg-white/6 px-4 py-4">
                <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">主机</p>
                <p className="mt-2 text-lg font-semibold text-white">{status.hostname}</p>
                <p className="mt-2 text-sm text-slate-300">{status.ros_distro}</p>
              </div>
              <div className="rounded-[26px] border border-white/10 bg-white/6 px-4 py-4">
                <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">当前阶段</p>
                <p className="mt-2 text-lg font-semibold text-white">{currentStep?.title ?? "全部完成"}</p>
                <p className="mt-2 text-sm text-slate-300">{currentStep?.description ?? "当前没有进行中的步骤。"}</p>
              </div>
            </div>
          </div>

          <div className="grid gap-3 md:grid-cols-2 xl:grid-cols-4">
            {[
              {
                icon: Workflow,
                title: "标定模式",
                value: status.tool.mode || "unknown",
                note: status.tool.checkerboard_spec
                  ? `CHECKERBOARD = ${status.tool.checkerboard_spec}`
                  : "尚未识别棋盘格规格",
              },
              {
                icon: Camera,
                title: "视觉模式",
                value: visionMode.mode,
                note: visionMode.stream_available ? "支持切换 stream" : "当前以 snapshot 为主",
              },
              {
                icon: Activity,
                title: "样本数量",
                value: String(metrics.calib_samples),
                note: `仓库 YAML ${status.files.repo_calib_count} 个`,
              },
              {
                icon: task?.running ? Clock3 : CheckCircle2,
                title: "任务状态",
                value: task?.running ? task.label : "空闲",
                note: task?.running ? `PID ${task.pid ?? "-"}` : "当前没有运行中的任务",
              },
            ].map((card) => (
              <Card key={card.title} className="border-white/8 bg-white/6 shadow-none backdrop-blur-sm">
                <CardContent className="p-5">
                  <div className="flex items-start justify-between gap-3">
                    <div>
                      <p className="text-sm text-slate-300">{card.title}</p>
                      <p className="mt-2 text-3xl font-semibold text-white">{card.value}</p>
                    </div>
                    <div className="inline-flex size-11 items-center justify-center rounded-2xl border border-white/10 bg-white/6">
                      <card.icon className="size-5 text-cyan-200" />
                    </div>
                  </div>
                  <p className="mt-4 text-sm leading-6 text-slate-400">{card.note}</p>
                </CardContent>
              </Card>
            ))}
          </div>
        </CardContent>
      </Card>

      <section className="grid gap-4 xl:grid-cols-[minmax(0,1.1fr)_minmax(340px,0.9fr)]">
        <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
          <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">版面分工</p>
          <div className="mt-4 grid gap-3">
            {[
              {
                title: "双目标定",
                description: "单独中心页处理向导、板信息、预检、采集与计算，不再与总览混排。",
              },
              {
                title: "视觉反馈",
                description: "单独中心页专注左右目画面与快照/流式模式切换，便于现场观察。",
              },
              {
                title: "验收日志",
                description: "把求解摘要、验收动作和任务日志集中到操作页，避免决策信息被埋没。",
              },
            ].map((item) => (
              <div key={item.title} className="rounded-3xl border border-white/8 bg-white/4 p-4">
                <p className="text-lg font-semibold text-white">{item.title}</p>
                <p className="mt-2 text-sm leading-6 text-slate-300">{item.description}</p>
              </div>
            ))}
          </div>
        </div>

        <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
          <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">运行摘要</p>
          <div className="mt-4 space-y-3">
            <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
              <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">工作区</p>
              <p className="mt-2 break-all font-mono text-sm text-slate-100">{status.paths.workspace_root}</p>
            </div>
            <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
              <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">视觉话题</p>
              <p className="mt-2 text-sm text-slate-100">
                左：{status.topics.left_image}
                <br />
                右：{status.topics.right_image}
              </p>
            </div>
            <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
              <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">任务统计</p>
              <p className="mt-2 text-sm text-slate-100">
                历史任务 {metrics.task_history_total} / 成功 {metrics.recent_success_count} / 失败 {metrics.recent_failure_count}
              </p>
            </div>
          </div>
        </div>
      </section>
    </section>
  )
}
