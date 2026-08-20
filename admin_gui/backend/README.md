# admin_gui backend

`backend/` 是 `E:\a_robot\rak-mercy\admin_gui` 的正式 Python 轻后端。

职责：

- 聚合 Deyes 标定状态
- 提供 `status / start / stop / logs / metrics`
- 提供标定表单与向导状态
- 提供双目视觉反馈接口

## 启动

本地开发：

```bash
cd E:\a_robot\rak-mercy\admin_gui\backend
python server.py --host 127.0.0.1 --port 8765
```

机器人部署：

```bash
cd /home/elephant/deyes_ws/src/admin_gui/backend
python3 server.py --host 0.0.0.0 --port 8765
```

## 环境变量

- `ADMIN_GUI_REPO_ROOT`
- `ADMIN_GUI_RUNTIME`
- `DEYES_REPO_ROOT`
- `DEYES_WORKSPACE`
- `DEYES_MERCURY_ROOT`
- `DEYES_LEFT_IMAGE_TOPIC`
- `DEYES_RIGHT_IMAGE_TOPIC`

## 首批接口

- `GET /api/status`
- `GET /api/logs`
- `POST /api/tasks/start`
- `POST /api/tasks/stop`
- `GET /api/metrics`
- `GET /api/calibration/form`
- `POST /api/calibration/form`
- `GET /api/calibration/wizard`
- `POST /api/calibration/wizard`
- `GET /api/vision/left`
- `GET /api/vision/right`
- `GET /api/vision/mode`
- `POST /api/vision/mode`

## 视觉反馈

- 默认模式：`snapshot`
- 预留模式：`stream`
- 首版以后端抓取 ROS 单帧 JPEG 形式服务左右图像
