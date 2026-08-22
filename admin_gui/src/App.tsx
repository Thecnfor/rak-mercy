import { useEffect, useMemo, useRef, useState } from "react"
import { Blocks, Camera, RefreshCw, ShieldAlert, Sparkles, Workflow } from "lucide-react"

import "./App.css"

import { CalibrationWizard } from "@/components/deyes/calibration-wizard"
import { OverviewPage } from "@/components/deyes/overview-page"
import { ReviewSummary } from "@/components/deyes/review-summary"
import { SidebarNav, type NavItemId } from "@/components/deyes/sidebar-nav"
import { SystemConsolePage } from "@/components/deyes/system-console-page"
import { TaskLogPanel } from "@/components/deyes/task-log-panel"
import { VisionPanel } from "@/components/deyes/vision-panel"
import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import {
  getMetrics,
  getStatus,
  getVisionMode,
  saveCalibrationForm,
  saveVisionMode,
  saveWizardState,
  startTask,
  stopTask,
} from "@/lib/api"
import type { AdminStatus, Metrics, TaskRecord, VisionMode } from "@/lib/types"

function emptyMetrics(): Metrics {
  return {
    task_history_total: 0,
    recent_success_count: 0,
    recent_failure_count: 0,
    calib_samples: 0,
  }
}

function currentTask(task: AdminStatus["task"]): TaskRecord | null {
  if (!task || !("task_id" in task)) return null
  return task as TaskRecord
}

function App() {
  const [activeItem, setActiveItem] = useState<NavItemId>("overview")
  const [status, setStatus] = useState<AdminStatus | null>(null)
  const [metrics, setMetrics] = useState<Metrics>(emptyMetrics())
  const [visionMode, setVisionMode] = useState<VisionMode>({
    mode: "snapshot",
    stream_available: false,
    refresh_ms: 2000,
  })
  const [boardId, setBoardId] = useState("")
  const [squareSize, setSquareSize] = useState("")
  const [reviewNote, setReviewNote] = useState("")
  const [leftVisionSrc, setLeftVisionSrc] = useState("/api/vision/left")
  const [rightVisionSrc, setRightVisionSrc] = useState("/api/vision/right")
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const hydratedForm = useRef(false)
  const hydratedReview = useRef(false)

  const task = useMemo(() => currentTask(status?.task ?? {}), [status])
  const pageMeta: Record<NavItemId, { eyebrow: string; title: string; description: string }> = {
    overview: {
      eyebrow: "Overview",
      title: "总览版面",
      description: "只看全局状态、当前阶段与关键风险，不再把所有操作都堆在首页。",
    },
    calibration: {
      eyebrow: "Calibration Center",
      title: "双目标定中心页",
      description: "从左侧栏进入的独立中心页，专门承接棋盘格双目标定流程。",
    },
    vision: {
      eyebrow: "Vision Feedback",
      title: "视觉反馈页",
      description: "独立展示左右目画面与视觉模式，方便现场观察与判断。",
    },
    operations: {
      eyebrow: "Operations",
      title: "验收日志页",
      description: "将求解摘要、验收动作与任务日志收拢到同一操作版面。",
    },
    system: {
      eyebrow: "System Console",
      title: "系统路径页",
      description: "统一查看机器人路径、接口与命令基座，不干扰标定主流程。",
    },
  }

  async function refreshAll() {
    const [nextStatus, nextMetrics, nextVisionMode] = await Promise.all([
      getStatus(),
      getMetrics(),
      getVisionMode(),
    ])
    setStatus(nextStatus)
    setMetrics(nextMetrics)
    setVisionMode(nextVisionMode)
    if (!hydratedForm.current) {
      setBoardId(nextStatus.calibration.form.board_id)
      setSquareSize(nextStatus.calibration.form.square_size_mm)
      hydratedForm.current = true
    }
    if (!hydratedReview.current) {
      setReviewNote(nextStatus.calibration.wizard.review.note)
      hydratedReview.current = true
    }
    const stamp = Date.now()
    setLeftVisionSrc(`/api/vision/left?ts=${stamp}`)
    setRightVisionSrc(`/api/vision/right?ts=${stamp}`)
  }

  useEffect(() => {
    refreshAll()
      .catch((cause: unknown) => {
        setError(cause instanceof Error ? cause.message : "初始化失败")
      })
      .finally(() => setLoading(false))

    const timer = window.setInterval(() => {
      refreshAll().catch(() => undefined)
    }, 3000)
    return () => window.clearInterval(timer)
  }, [])

  async function handleRefresh() {
    try {
      setError(null)
      await refreshAll()
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : "刷新失败")
    }
  }

  async function handleStartTask(taskId: string) {
    try {
      setError(null)
      await startTask(taskId)
      await refreshAll()
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : "启动任务失败")
    }
  }

  async function handleStopTask() {
    try {
      setError(null)
      await stopTask()
      await refreshAll()
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : "停止任务失败")
    }
  }

  async function handleSaveForm() {
    if (!boardId.trim()) {
      setError("请先填写板编号。")
      return
    }
    if (!(Number(squareSize) > 0)) {
      setError("请填写大于 0 的单方格边长。")
      return
    }
    try {
      setError(null)
      await saveCalibrationForm({
        board_id: boardId.trim(),
        square_size_mm: squareSize.trim(),
      })
      await saveWizardState({ current_step: "capture" })
      hydratedForm.current = false
      await refreshAll()
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : "保存板信息失败")
    }
  }

  async function handleReviewPass(passed: boolean) {
    try {
      setError(null)
      await saveWizardState({
        current_step: passed ? "baseline" : "capture",
        review_passed: passed,
        review_note: reviewNote.trim(),
      })
      hydratedReview.current = false
      await refreshAll()
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : "保存验收状态失败")
    }
  }

  async function handleModeChange(mode: "snapshot" | "stream") {
    try {
      setError(null)
      await saveVisionMode(mode)
      await refreshAll()
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : "切换视觉模式失败")
    }
  }

  if (loading) {
    return (
      <div className="flex min-h-screen items-center justify-center px-6">
        <div className="rounded-[28px] border border-white/10 bg-card/90 px-8 py-6 text-sm text-slate-200 backdrop-blur-xl">
          正在连接 admin_gui 后端与 Deyes 标定状态...
        </div>
      </div>
    )
  }

  if (!status) {
    return (
      <div className="flex min-h-screen items-center justify-center px-6">
        <div className="max-w-xl rounded-[28px] border border-rose-300/20 bg-rose-400/8 px-8 py-6 text-sm leading-7 text-rose-100">
          {error ?? "尚未获取到后端状态。请先启动 admin_gui/backend/server.py 或检查 `/api/status`。"}
        </div>
      </div>
    )
  }

  return (
    <div className="min-h-screen p-4 md:p-6">
      <div className="mx-auto grid max-w-[1700px] gap-4 xl:grid-cols-[300px_minmax(0,1fr)]">
        <aside className="deyes-sidebar-card xl:sticky xl:top-6 xl:self-start">
          <div className="space-y-6">
            <div className="space-y-4">
              <div className="inline-flex size-13 items-center justify-center rounded-[24px] border border-white/10 bg-white/6">
                <Blocks className="size-6 text-cyan-200" />
              </div>
              <div className="space-y-3">
                <Badge variant="outline" className="border-cyan-300/18 bg-cyan-400/10 text-cyan-100">
                  Formal Entry / 正式入口
                </Badge>
                <div>
                  <h1 className="text-2xl font-semibold text-white">admin_gui / 项目控制台</h1>
                  <p className="mt-3 text-sm leading-6 text-slate-300">
                    版面已拆为多中心页，双目标定从左侧栏进入独立中心页，视觉、日志与系统路径各自分屏处理。
                  </p>
                </div>
              </div>
            </div>

            <SidebarNav activeItem={activeItem} onSelect={setActiveItem} />

            <div className="space-y-3">
              {[
                {
                  icon: Camera,
                  label: "当前页",
                  value: pageMeta[activeItem].title,
                },
                {
                  icon: Workflow,
                  label: "标定模式",
                  value: status.tool.mode === "checkerboard" ? "棋盘格流程已识别" : "工具状态待确认",
                },
                {
                  icon: ShieldAlert,
                  label: "任务状态",
                  value: task?.running ? `${task.label} / 运行中` : "当前空闲",
                },
              ].map((item) => (
                <div key={item.label} className="rounded-3xl border border-white/8 bg-white/4 p-4">
                  <div className="flex items-start gap-3">
                    <div className="inline-flex size-10 items-center justify-center rounded-2xl bg-white/6">
                      <item.icon className="size-5 text-cyan-200" />
                    </div>
                    <div>
                      <p className="text-sm text-slate-400">{item.label}</p>
                      <p className="mt-1 text-sm leading-6 text-white">{item.value}</p>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </aside>

        <main className="space-y-4">
          <Card className="border-white/10 bg-[linear-gradient(145deg,rgba(8,14,24,0.98),rgba(9,22,36,0.96))] shadow-2xl shadow-black/25">
            <CardContent className="flex flex-col gap-5 p-6 md:p-7 xl:flex-row xl:items-end xl:justify-between">
              <div className="space-y-4">
                <Badge className="border-none bg-cyan-400/12 text-cyan-100">{pageMeta[activeItem].eyebrow}</Badge>
                <div>
                  <h2 className="text-3xl font-semibold tracking-tight text-white md:text-4xl">
                    {pageMeta[activeItem].title}
                  </h2>
                  <p className="mt-3 max-w-4xl text-sm leading-7 text-slate-300 md:text-base">
                    {pageMeta[activeItem].description}
                  </p>
                </div>
              </div>

              <div className="flex flex-wrap items-center gap-3">
                <div className="rounded-full border border-white/10 bg-white/4 px-4 py-2 text-sm text-slate-200">
                  {status.hostname} / {status.ros_distro}
                </div>
                <button
                  type="button"
                  onClick={handleRefresh}
                  className="inline-flex items-center gap-2 rounded-full border border-white/10 bg-white px-4 py-2 text-sm font-medium text-slate-950 transition hover:bg-slate-100"
                >
                  <RefreshCw className="size-4" />
                  刷新状态
                </button>
              </div>
            </CardContent>
          </Card>

          {error ? (
            <div className="rounded-[24px] border border-amber-300/18 bg-amber-400/10 px-4 py-3 text-sm leading-6 text-amber-50">
              {error}
            </div>
          ) : null}

          {activeItem === "overview" && (
            <OverviewPage status={status} metrics={metrics} task={task} visionMode={visionMode} />
          )}

          {activeItem === "calibration" && (
            <div className="space-y-4">
              <Card className="overflow-hidden border-white/10 bg-[linear-gradient(135deg,rgba(8,17,30,0.98),rgba(9,30,48,0.92)_40%,rgba(6,15,25,0.96))] shadow-2xl shadow-black/30">
                <CardContent className="space-y-6 p-6 md:p-8">
                  <div className="flex flex-col gap-4 xl:flex-row xl:items-end xl:justify-between">
                    <div className="space-y-4">
                      <Badge className="border-none bg-emerald-400/12 text-emerald-100">
                        Deyes / Calibration Center
                      </Badge>
                      <div className="space-y-3">
                        <h3 className="max-w-5xl text-4xl leading-tight font-semibold tracking-tight text-white md:text-5xl">
                          双目标定现在是独立中心页，不再和视觉、日志、系统路径挤在同一屏。
                        </h3>
                        <p className="max-w-4xl text-base leading-7 text-slate-300 md:text-lg">
                          这个页面只处理棋盘格 `9x6` 的预检、板信息、采集、计算与切换阶段，保证操作视线集中。
                        </p>
                      </div>
                    </div>

                    <div className="grid gap-3 sm:grid-cols-2">
                      <div className="rounded-[26px] border border-white/10 bg-white/6 px-4 py-4">
                        <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">当前步骤</p>
                        <p className="mt-2 text-lg font-semibold text-white">
                          {status.calibration.wizard.steps.find((step) => step.status === "current")?.title ?? "全部完成"}
                        </p>
                      </div>
                      <div className="rounded-[26px] border border-white/10 bg-white/6 px-4 py-4">
                        <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">板信息</p>
                        <p className="mt-2 text-lg font-semibold text-white">
                          {boardId.trim() || "待填写"} / {squareSize.trim() ? `${squareSize.trim()} mm` : "待填写"}
                        </p>
                      </div>
                    </div>
                  </div>
                </CardContent>
              </Card>

              <div className="grid gap-4 xl:grid-cols-[minmax(0,1.3fr)_minmax(340px,0.7fr)]">
                <CalibrationWizard
                  commands={status.commands}
                  form={{
                    ...status.calibration.form,
                    board_id: boardId,
                    square_size_mm: squareSize,
                  }}
                  wizard={status.calibration.wizard}
                  guidance={status.checkerboard_guidance}
                  onStartTask={handleStartTask}
                  onSaveForm={handleSaveForm}
                />

                <div className="space-y-4">
                  <Card className="border-white/10 bg-card/90 backdrop-blur-xl">
                    <CardHeader>
                      <CardTitle className="text-white">板信息录入</CardTitle>
                      <CardDescription>录入真实板编号和单方格边长后，向导才允许进入采集阶段。</CardDescription>
                    </CardHeader>
                    <CardContent className="space-y-4">
                      <label className="block">
                        <span className="mb-2 block text-sm text-slate-300">板编号</span>
                        <input
                          value={boardId}
                          onChange={(event) => setBoardId(event.target.value)}
                          className="w-full rounded-[22px] border border-white/10 bg-white/4 px-4 py-3 text-sm text-white outline-none placeholder:text-slate-500"
                          placeholder="checkerboard_9x6_board01"
                        />
                      </label>
                      <label className="block">
                        <span className="mb-2 block text-sm text-slate-300">单方格边长 (mm)</span>
                        <input
                          value={squareSize}
                          onChange={(event) => setSquareSize(event.target.value)}
                          className="w-full rounded-[22px] border border-white/10 bg-white/4 px-4 py-3 text-sm text-white outline-none placeholder:text-slate-500"
                          placeholder="例如 24.0"
                        />
                      </label>
                      <button
                        type="button"
                        onClick={handleSaveForm}
                        className="inline-flex items-center gap-2 rounded-full bg-white px-4 py-2.5 text-sm font-medium text-slate-950 transition hover:bg-slate-100"
                      >
                        <Sparkles className="size-4" />
                        保存板信息
                      </button>
                    </CardContent>
                  </Card>

                  <Card className="border-white/10 bg-card/90 backdrop-blur-xl">
                    <CardHeader>
                      <CardTitle className="text-white">当前标定约束</CardTitle>
                      <CardDescription>把现场必须确认的信息收敛在右侧，减少切屏记忆成本。</CardDescription>
                    </CardHeader>
                    <CardContent className="space-y-3 text-sm text-slate-300">
                      {[
                        {
                          label: "固定规格",
                          value: `${status.checkerboard_guidance.inner_corners} / ${status.checkerboard_guidance.print_scale}`,
                        },
                        { label: "工作区", value: status.paths.workspace_root },
                        { label: "采样目录", value: `${status.paths.calib_dir} / ${status.files.calib_samples} 个文件` },
                        { label: "标定工具", value: `${status.paths.mercury_root}/calibrate_stereo.py` },
                      ].map((item) => (
                        <div key={item.label} className="rounded-3xl border border-white/8 bg-white/4 p-4">
                          <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">{item.label}</p>
                          <p className="mt-2 break-all font-mono text-sm text-slate-100">{item.value}</p>
                        </div>
                      ))}
                    </CardContent>
                  </Card>
                </div>
              </div>
            </div>
          )}

          {activeItem === "vision" && (
            <div className="space-y-4">
              <Card className="border-white/10 bg-card/90 backdrop-blur-xl">
                <CardContent className="space-y-5 p-6">
                  <div className="flex flex-col gap-4 xl:flex-row xl:items-end xl:justify-between">
                    <div className="space-y-3">
                      <Badge className="border-none bg-sky-400/12 text-sky-100">Stereo Vision Workspace</Badge>
                      <h3 className="text-3xl font-semibold tracking-tight text-white">双目视觉反馈独立版面</h3>
                      <p className="max-w-4xl text-sm leading-7 text-slate-300">
                        单独查看左右目快照或后续流式画面，避免在标定中心页里被长日志和路径信息挤压观察区域。
                      </p>
                    </div>
                    <div className="grid gap-3 sm:grid-cols-2">
                      <div className="rounded-[24px] border border-white/10 bg-white/5 px-4 py-4">
                        <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">左目话题</p>
                        <p className="mt-2 break-all text-sm text-slate-100">{status.topics.left_image}</p>
                      </div>
                      <div className="rounded-[24px] border border-white/10 bg-white/5 px-4 py-4">
                        <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">右目话题</p>
                        <p className="mt-2 break-all text-sm text-slate-100">{status.topics.right_image}</p>
                      </div>
                    </div>
                  </div>
                </CardContent>
              </Card>

              <VisionPanel
                leftSrc={leftVisionSrc}
                rightSrc={rightVisionSrc}
                topics={status.topics}
                visionMode={visionMode}
                onModeChange={handleModeChange}
              />
            </div>
          )}

          {activeItem === "operations" && (
            <div className="space-y-4">
              <Card className="border-white/10 bg-card/90 backdrop-blur-xl">
                <CardContent className="space-y-4 p-6">
                  <Badge className="border-none bg-amber-400/12 text-amber-100">Review / Logs / Actions</Badge>
                  <div className="space-y-3">
                    <h3 className="text-3xl font-semibold tracking-tight text-white">验收与日志独立版面</h3>
                    <p className="max-w-4xl text-sm leading-7 text-slate-300">
                      求解摘要、验收结论和任务日志集中到同一中心页，便于在计算完成后快速做通过/重采判断。
                    </p>
                  </div>
                </CardContent>
              </Card>

              <ReviewSummary
                wizard={status.calibration.wizard}
                metrics={metrics}
                reviewNote={reviewNote}
                onReviewNoteChange={setReviewNote}
                onReviewPass={handleReviewPass}
              />

              <TaskLogPanel task={task} logTail={status.log_tail} onStop={handleStopTask} />
            </div>
          )}

          {activeItem === "system" && <SystemConsolePage status={status} />}
        </main>
      </div>
    </div>
  )
}

export default App
