import type { CommandSpec, WizardState } from "@/lib/types"

export const STEP_COMMANDS: Record<string, string[]> = {
  precheck: ["precheck_1280", "precheck_720"],
  board: [],
  capture: ["capture"],
  compute: ["compute"],
  review: [],
  baseline: ["stereo_image_proc", "sgbm"],
}

export function commandMap(commands: CommandSpec[]) {
  return new Map(commands.map((command) => [command.id, command]))
}

export function stepStatusLabel(status: WizardState["steps"][number]["status"]) {
  if (status === "completed") return "已完成"
  if (status === "current") return "进行中"
  return "待执行"
}

export function reviewTone(passed: boolean | null) {
  if (passed === true) return "success"
  if (passed === false) return "warning"
  return "neutral"
}
