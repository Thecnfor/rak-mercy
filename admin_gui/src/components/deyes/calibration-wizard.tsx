import type { CalibrationForm, CommandSpec, WizardState } from "@/lib/types"
import { STEP_COMMANDS, commandMap, stepStatusLabel } from "@/lib/deyes"

type CalibrationWizardProps = {
  commands: CommandSpec[]
  form: CalibrationForm
  wizard: WizardState
  guidance: {
    inner_corners: string
    print_scale: string
    required_notes: string[]
  }
  onStartTask: (taskId: string) => void
  onSaveForm: () => void
}

function statusClass(status: WizardState["steps"][number]["status"]) {
  if (status === "completed") return "border-emerald-300/20 bg-emerald-400/6 text-emerald-100"
  if (status === "current") return "border-cyan-300/25 bg-cyan-400/8 text-cyan-100"
  return "border-white/8 bg-white/4 text-slate-300"
}

export function CalibrationWizard({
  commands,
  form,
  wizard,
  guidance,
  onStartTask,
  onSaveForm,
}: CalibrationWizardProps) {
  const commandsById = commandMap(commands)

  return (
    <section className="grid gap-4 xl:grid-cols-[minmax(0,1.2fr)_minmax(360px,0.8fr)]">
      <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
        <div className="mb-4 flex items-end justify-between gap-3">
          <div>
            <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">标定向导</p>
            <h3 className="mt-2 text-2xl font-semibold text-white">Deyes / 标定</h3>
            <p className="mt-2 max-w-3xl text-sm leading-6 text-slate-300">
              用棋盘格 `9x6` 串联预检、板信息、采集、计算、验收与复验，避免手工记忆 ROS 和 SSH 命令。
            </p>
          </div>
          <div className="rounded-full border border-white/10 bg-white/4 px-4 py-2 text-sm text-slate-200">
            当前步骤：{wizard.steps.find((step) => step.status === "current")?.title ?? "全部完成"}
          </div>
        </div>

        <div className="grid gap-3">
          {wizard.steps.map((step) => {
            const stepCommands = STEP_COMMANDS[step.id] ?? []
            return (
              <div
                key={step.id}
                className={`rounded-[26px] border p-4 transition ${statusClass(step.status)}`}
              >
                <div className="flex flex-col gap-3 md:flex-row md:items-start md:justify-between">
                  <div>
                    <p className="text-lg font-semibold text-white">{step.title}</p>
                    <p className="mt-2 text-sm leading-6 text-slate-300">{step.description}</p>
                  </div>
                  <span className="rounded-full border border-white/10 bg-black/20 px-3 py-1 text-xs tracking-[0.18em] uppercase">
                    {stepStatusLabel(step.status)}
                  </span>
                </div>

                <div className="mt-4 flex flex-wrap gap-3">
                  {step.id === "board" && (
                    <button
                      type="button"
                      onClick={onSaveForm}
                      className="rounded-full bg-white px-4 py-2 text-sm font-medium text-slate-950 transition hover:bg-slate-100"
                    >
                      保存板信息
                    </button>
                  )}

                  {stepCommands.map((taskId) => {
                    const command = commandsById.get(taskId)
                    if (!command) return null
                    const primary = taskId === "precheck_1280" || taskId === "capture" || taskId === "compute"
                    return (
                      <button
                        key={taskId}
                        type="button"
                        onClick={() => onStartTask(taskId)}
                        className={
                          primary
                            ? "rounded-full bg-white px-4 py-2 text-sm font-medium text-slate-950 transition hover:bg-slate-100"
                            : "rounded-full border border-white/10 bg-white/4 px-4 py-2 text-sm text-slate-100 transition hover:bg-white/10"
                        }
                      >
                        {command.label}
                      </button>
                    )
                  })}
                </div>
              </div>
            )
          })}
        </div>
      </div>

      <div className="space-y-4">
        <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
          <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">板信息确认</p>
          <div className="mt-4 grid gap-3">
            <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
              <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">板编号</p>
              <p className="mt-2 font-mono text-sm text-slate-100">{form.board_id || "未填写"}</p>
            </div>
            <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
              <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">单方格边长</p>
              <p className="mt-2 font-mono text-sm text-slate-100">
                {form.square_size_mm ? `${form.square_size_mm} mm` : "未填写"}
              </p>
            </div>
            <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
              <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">固定条件</p>
              <p className="mt-2 text-sm text-slate-100">
                内角点：{guidance.inner_corners} / 打印比例：{guidance.print_scale}
              </p>
            </div>
          </div>
        </div>

        <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
          <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">执行提示</p>
          <ul className="mt-4 space-y-3">
            {guidance.required_notes.map((item) => (
              <li key={item} className="rounded-3xl border border-white/8 bg-white/4 px-4 py-3 text-sm leading-6 text-slate-200">
                {item}
              </li>
            ))}
          </ul>
        </div>
      </div>
    </section>
  )
}
