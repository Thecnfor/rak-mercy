import type { Metrics, WizardState } from "@/lib/types"

type ReviewSummaryProps = {
  wizard: WizardState
  metrics: Metrics
  reviewNote: string
  onReviewNoteChange: (value: string) => void
  onReviewPass: (passed: boolean) => void
}

export function ReviewSummary({
  wizard,
  metrics,
  reviewNote,
  onReviewNoteChange,
  onReviewPass,
}: ReviewSummaryProps) {
  const passed = wizard.review.passed
  const summary = wizard.compute_summary

  return (
    <section className="grid gap-4 xl:grid-cols-[minmax(0,0.95fr)_minmax(0,1.05fr)]">
      <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
        <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">求解摘要</p>
        <div className="mt-4 grid gap-3">
          <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
            <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">reproj_error</p>
            <p className="mt-2 text-2xl font-semibold text-white">
              {summary.reproj_error === null ? "未提取" : summary.reproj_error.toFixed(3)}
            </p>
          </div>
          <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
            <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">YAML 路径</p>
            <p className="mt-2 break-all font-mono text-sm text-slate-200">
              {summary.yaml_path ?? "尚未在日志中识别到 YAML 路径"}
            </p>
          </div>
          <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
            <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">核心矩阵</p>
            <p className="mt-2 text-sm text-slate-200">
              {summary.has_core_matrices ? "K1 / D1 / K2 / D2 / R / T 已识别" : "尚未确认核心矩阵输出"}
            </p>
          </div>
        </div>
      </div>

      <div className="rounded-[28px] border border-white/10 bg-card/90 p-5 backdrop-blur-xl">
        <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">验收与指标</p>
        <div className="mt-4 grid gap-3 md:grid-cols-3">
          <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
            <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">历史任务</p>
            <p className="mt-2 text-2xl font-semibold text-white">{metrics.task_history_total}</p>
          </div>
          <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
            <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">成功任务</p>
            <p className="mt-2 text-2xl font-semibold text-emerald-200">{metrics.recent_success_count}</p>
          </div>
          <div className="rounded-3xl border border-white/8 bg-white/4 p-4">
            <p className="text-xs tracking-[0.18em] text-slate-500 uppercase">样本数量</p>
            <p className="mt-2 text-2xl font-semibold text-white">{metrics.calib_samples}</p>
          </div>
        </div>

        <div className="mt-4 rounded-3xl border border-white/8 bg-black/12 p-4">
          <p className="text-sm leading-6 text-slate-200">
            停止条件：`reproj_error &gt; 0.50 px`、分辨率不一致、棋盘格规格不一致、核心矩阵缺失。
          </p>
          <p className="mt-3 text-sm text-slate-300">
            当前验收状态：
            <span className="ml-2 font-medium text-white">
              {passed === true ? "已通过" : passed === false ? "需重采" : "尚未确认"}
            </span>
          </p>
        </div>

        <textarea
          value={reviewNote}
          onChange={(event) => onReviewNoteChange(event.target.value)}
          className="mt-4 min-h-28 w-full rounded-[24px] border border-white/10 bg-white/4 px-4 py-3 text-sm text-white outline-none ring-0 placeholder:text-slate-500"
          placeholder="填写验收备注、阻塞原因或重采建议。"
        />

        <div className="mt-4 flex flex-wrap gap-3">
          <button
            type="button"
            onClick={() => onReviewPass(true)}
            className="rounded-full bg-emerald-300 px-5 py-2.5 text-sm font-medium text-slate-950 transition hover:bg-emerald-200"
          >
            标记验收通过
          </button>
          <button
            type="button"
            onClick={() => onReviewPass(false)}
            className="rounded-full border border-white/10 bg-white/4 px-5 py-2.5 text-sm font-medium text-slate-100 transition hover:bg-white/10"
          >
            标记需重采
          </button>
        </div>
      </div>
    </section>
  )
}
