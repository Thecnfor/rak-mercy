export type NavItemId = "overview" | "calibration" | "vision" | "operations" | "system"

type SidebarNavProps = {
  activeItem: NavItemId
  onSelect: (item: NavItemId) => void
}

const groups: Array<{
  title: string
  items: Array<{
    id: NavItemId
    title: string
    description: string
  }>
}> = [
  {
    title: "Deyes",
    items: [
      {
        id: "overview",
        title: "总览",
        description: "查看主机状态、指标摘要与当前阶段",
      },
      {
        id: "calibration",
        title: "双目标定",
        description: "作为独立中心页进入棋盘格标定流程",
      },
      {
        id: "vision",
        title: "视觉反馈",
        description: "单独查看左右目画面与视觉模式",
      },
      {
        id: "operations",
        title: "验收日志",
        description: "集中处理求解摘要、验收动作与任务日志",
      },
    ],
  },
  {
    title: "Platform",
    items: [
      {
        id: "system",
        title: "系统路径",
        description: "查看机器人路径、接口与运行基座",
      },
    ],
  },
]

export function SidebarNav({ activeItem, onSelect }: SidebarNavProps) {
  return (
    <div className="space-y-6">
      {groups.map((group) => (
        <div key={group.title} className="space-y-3">
          <p className="text-xs font-medium tracking-[0.22em] text-slate-400 uppercase">{group.title}</p>
          <div className="space-y-2">
            {group.items.map((item) => {
              const active = activeItem === item.id
              return (
                <button
                  key={item.id}
                  type="button"
                  onClick={() => onSelect(item.id)}
                  className={`deyes-sidebar-item w-full rounded-3xl border px-4 py-4 text-left transition ${
                    active
                      ? "border-cyan-300/25 bg-cyan-400/10 shadow-lg shadow-cyan-950/20"
                      : "border-white/8 bg-white/4 hover:border-white/14 hover:bg-white/6"
                  }`}
                >
                  <p className="text-sm font-medium text-white">{item.title}</p>
                  <p className="mt-1 text-sm leading-6 text-slate-300">{item.description}</p>
                </button>
              )
            })}
          </div>
        </div>
      ))}
    </div>
  )
}
