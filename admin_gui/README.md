# admin_gui

面向 `E:\a_robot\rak-mercy` 全项目的独立管理网站，用来承接：

- 实时状态监控
- 调试与测试入口
- 任务编排与日志回放
- 传感器面板与人工接管触控
- Deyes、双臂控制、底盘导航、系统服务的统一可视化入口

当前版本已经将 `Deyes -> 标定` 作为第一个真实接入 domain 落到外层 `admin_gui`，并配套提供 `backend/` Python 轻后端。旧版实现已归档到工作区外层 `temp`，不再作为正式入口。

## 技术栈

- `React 19`
- `TypeScript`
- `Vite`
- `Tailwind CSS v4`
- `shadcn/ui`
- `lucide-react`

## 当前页面能力

- `Deyes -> 标定` 左侧栏入口
- 棋盘格 `9x6` 标定向导
- 双目视觉反馈：快照模式 + 流式模式预留
- `status / start / stop / logs / metrics / calibration / vision` 首批接口接线
- 可扩展的 `shadcn` 组件基础

## 本地运行

```bash
cd E:\a_robot\rak-mercy\admin_gui
npm install
npm run dev
```

另起一个终端启动正式后端：

```bash
cd E:\a_robot\rak-mercy\admin_gui\backend
python server.py --host 127.0.0.1 --port 8765
```

构建生产包：

```bash
npm run build
```

## 目录说明

```text
admin_gui/
├── backend/              # 正式 Python 轻后端
├── src/
│   ├── components/ui/    # shadcn 组件
│   ├── components/deyes/ # Deyes 标定页面组件
│   ├── lib/              # API 与类型定义
│   ├── App.tsx           # 正式 Deyes 标定入口
│   ├── index.css         # 全局主题与 Tailwind 入口
│   └── main.tsx          # 应用入口
├── components.json      # shadcn 配置
├── package.json
└── vite.config.ts
```

## 下一步接管路线

1. 继续增强 `backend/`，把 `vision stream` 从预留模式做成真实可用模式。
2. 在 Deyes domain 基础上，为双臂、底盘、系统服务拆出独立页面。
3. 为日志、状态与任务执行补充 SSE / WebSocket 实时推送。
4. 将任务总线、日志审计与安全互锁统一进项目级任务模型。
5. 用真实 ROS 2 / 系统数据逐步替换剩余演示态区域。

## 说明

- `admin_gui/` 是面向整个项目的唯一正式前端入口。
- `backend/` 是外层 `admin_gui` 的正式后端入口。
- 旧版 `Deyes/admin_gui` 已归档到 `E:/a_robot/temp/rak-mercy-baseline/archive/Deyes-admin_gui-legacy/`，不再作为正式入口使用。
- 你提到的 `shadcn` 已完成接入。
- `impeccab`、`designMD collection` 当前先按设计参考源处理，待你给出明确包名、模板源或仓库后可继续精确接入。
