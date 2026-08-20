export type TaskRecord = {
  task_id: string
  label: string
  command: string
  log_path: string
  started_at: string
  pid: number | null
  running: boolean
  return_code?: number | null
  finished_at?: string | null
}

export type CommandSpec = {
  id: string
  label: string
  description: string
  command: string
}

export type CalibrationForm = {
  board_id: string
  square_size_mm: string
  inner_corners: string
  print_scale: string
}

export type WizardStep = {
  id: string
  title: string
  description: string
  status: "completed" | "current" | "pending"
}

export type ComputeSummary = {
  available: boolean
  reproj_error: number | null
  yaml_path: string | null
  has_core_matrices: boolean
  source_log: string | null
}

export type WizardState = {
  current_step: string
  steps: WizardStep[]
  review: {
    passed: boolean | null
    note: string
  }
  form_ready: boolean
  compute_summary: ComputeSummary
}

export type Metrics = {
  task_history_total: number
  recent_success_count: number
  recent_failure_count: number
  calib_samples: number
}

export type VisionMode = {
  mode: "snapshot" | "stream"
  stream_available: boolean
  refresh_ms: number
}

export type AdminStatus = {
  hostname: string
  platform: string
  python: string
  time: string
  ros_distro: string
  paths: {
    repo_root: string
    workspace_root: string
    mercury_root: string
    repo_calib_dir: string
    calib_dir: string
    placeholder_calib: string
  }
  files: {
    calib_tool_exists: boolean
    calib_dir_exists: boolean
    placeholder_calib_exists: boolean
    repo_calib_count: number
    repo_calibs: string[]
    calib_samples: number
  }
  tool: {
    exists: boolean
    mode: string
    checkerboard_supported: boolean
    charuco_supported: boolean
    checkerboard_spec: string | null
  }
  commands: CommandSpec[]
  task: TaskRecord | Record<string, never>
  task_history: TaskRecord[]
  log_tail: string
  checkerboard_guidance: {
    inner_corners: string
    print_scale: string
    required_notes: string[]
  }
  topics: {
    left_image: string
    right_image: string
  }
  calibration: {
    form: CalibrationForm
    wizard: WizardState
  }
  vision: VisionMode
  metrics: Metrics
}
