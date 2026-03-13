# 盲点 (Blind Spot)

一款点击式视觉推理小说游戏。

## 🎮 游戏简介

玩家扮演盲人，进入各种犯罪现场。场景中充满疑点，NPC 会与玩家对话试探。玩家必须：
- **隐藏身份**：不能表现出能看到东西
- **做出选择**：对话选项会影响 NPC 的怀疑值
- **推进剧情**：怀疑值超过阈值则游戏失败

## 🎯 核心玩法

```
加载关卡 → 展示场景（背景 + 可点击疑点）→ NPC 对话 → 
玩家选择选项 → 怀疑值变化 → 推进剧情 → 结局判定
```

## 📁 项目结构

```
simplegame/
├── README.md              # 本文件
├── package.json           # 项目配置 + 打包脚本
├── electron/              # Electron 打包相关
├── src/                   # 游戏源代码
│   ├── index.html
│   ├── css/
│   └── js/
├── src/data/levels/       # 关卡配置（JSON）
├── assets/                # 美术资源
└── docs/                  # 设计文档
```

## 🚀 快速开始

### 开发模式
```bash
npm install
npm run dev
# 浏览器打开 http://localhost:3000
```

### 部署到 Vercel
```bash
# 首次部署
vercel --prod

# 后续更新
vercel --prod

# 遇到缓存问题时
vercel --prod --force
```

**线上地址：** https://simplegame-rosy.vercel.app

### 打包
```bash
npm run build:win    # Windows
npm run build:mac    # Mac
npm run build:linux  # Linux
```

## 📖 文档

- [设计文档](docs/design.md) - 完整游戏设计
- [配置规范](docs/config-spec.md) - 关卡 JSON 配置格式
- [开发计划](docs/roadmap.md) - 开发进度追踪
- [AI 配置提示词](tools/ai-config-prompt.md) - 用 AI 生成关卡

## 🛠️ 技术栈

- **核心**：HTML + CSS + JavaScript（原生）
- **打包**：Electron（PC 桌面应用）
- **关卡**：JSON 配置（独立可配置，支持 AI 生成）

## 🎯 设计原则

1. **关卡完全独立** - 新增关卡只需添加 JSON，无需改代码
2. **AI 可配置** - 结构化配置，AI 可根据自然语言生成
3. **热加载** - 开发时修改配置立即生效

---

*开发中...*
