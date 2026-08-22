import type { AdminStatus, CalibrationForm, Metrics, VisionMode, WizardState } from "@/lib/types"

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(path, {
    headers: {
      "Content-Type": "application/json",
      ...(init?.headers ?? {}),
    },
    ...init,
  })

  const contentType = response.headers.get("content-type") ?? ""
  const payload = contentType.includes("application/json")
    ? await response.json()
    : await response.text()

  if (!response.ok) {
    const message =
      typeof payload === "object" && payload && "error" in payload ? String(payload.error) : "请求失败"
    throw new Error(message)
  }

  return payload as T
}

export async function getStatus() {
  return request<AdminStatus>("/api/status")
}

export async function getMetrics() {
  return request<Metrics>("/api/metrics")
}

export async function getLogs(logPath?: string) {
  const query = logPath ? `?path=${encodeURIComponent(logPath)}` : ""
  return request<{ log: string }>(`/api/logs${query}`)
}

export async function startTask(taskId: string) {
  return request<{ ok: boolean }>("/api/tasks/start", {
    method: "POST",
    body: JSON.stringify({ task_id: taskId }),
  })
}

export async function stopTask() {
  return request<{ stopped: boolean; message?: string }>("/api/tasks/stop", {
    method: "POST",
  })
}

export async function getCalibrationForm() {
  return request<CalibrationForm>("/api/calibration/form")
}

export async function saveCalibrationForm(payload: Pick<CalibrationForm, "board_id" | "square_size_mm">) {
  return request<CalibrationForm>("/api/calibration/form", {
    method: "POST",
    body: JSON.stringify(payload),
  })
}

export async function getWizardState() {
  return request<WizardState>("/api/calibration/wizard")
}

export async function saveWizardState(payload: {
  current_step?: string
  review_passed?: boolean | null
  review_note?: string
}) {
  return request<WizardState>("/api/calibration/wizard", {
    method: "POST",
    body: JSON.stringify(payload),
  })
}

export async function getVisionMode() {
  return request<VisionMode>("/api/vision/mode")
}

export async function saveVisionMode(mode: "snapshot" | "stream") {
  return request<VisionMode>("/api/vision/mode", {
    method: "POST",
    body: JSON.stringify({ mode }),
  })
}
