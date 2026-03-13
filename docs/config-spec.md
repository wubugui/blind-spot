# 关卡配置规范

## 📋 配置文件格式

所有关卡配置使用 JSON 格式，存放于 `src/data/levels/` 目录。

文件名格式：`level-XX.json`（XX 为关卡编号，如 `level-01.json`）

---

## 📐 JSON Schema

### 完整结构

```json
{
  "version": "1.0",
  "meta": { ... },
  "scene": { ... },
  "hotspots": [ ... ],
  "dialogues": { ... },
  "rules": { ... }
}
```

---

## 🔍 字段详解

### 1. version（必填）

配置文件版本号，用于兼容性检查。

```json
{
  "version": "1.0"
}
```

---

### 2. meta（必填）

关卡元信息。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `id` | string | ✅ | 关卡唯一标识，如 `"level-01"` |
| `name` | string | ✅ | 关卡名称，如 `"出租屋"` |
| `description` | string | ✅ | 关卡简介 |
| `difficulty` | number | ✅ | 难度等级（1-5） |
| `estimatedTime` | number | ✅ | 预计通关时间（秒） |

```json
{
  "meta": {
    "id": "level-01",
    "name": "出租屋",
    "description": "女人杀了人，盲人钢琴老师来调音",
    "difficulty": 1,
    "estimatedTime": 180
  }
}
```

---

### 3. scene（必填）

场景信息。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `background` | string | ✅ | 背景图片路径（相对于 assets） |
| `npc` | object | ✅ | NPC 信息 |
| `player` | object | ✅ | 玩家身份 |

#### 3.1 npc 对象

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `id` | string | ✅ | NPC 标识 |
| `name` | string | ✅ | NPC 名称 |
| `avatar` | string | ❌ | NPC 头像图片路径 |
| `description` | string | ✅ | NPC 描述 |

#### 3.2 player 对象

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `role` | string | ✅ | 玩家角色 |
| `background` | string | ✅ | 玩家背景故事 |

```json
{
  "scene": {
    "background": "backgrounds/rental-room.jpg",
    "npc": {
      "id": "woman",
      "name": "房东太太",
      "avatar": "characters/woman.png",
      "description": "30 多岁，神色紧张，手上有血迹"
    },
    "player": {
      "role": "钢琴调音师",
      "background": "你是盲人，被叫来调钢琴"
    }
  }
}
```

---

### 4. hotspots（必填）

疑点列表，数组格式。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `id` | string | ✅ | 疑点唯一标识 |
| `x` | number | ✅ | X 坐标（0-1 相对值） |
| `y` | number | ✅ | Y 坐标（0-1 相对值） |
| `width` | number | ✅ | 宽度（0-1 相对值） |
| `height` | number | ✅ | 高度（0-1 相对值） |
| `description` | string | ✅ | 点击后的描述文本 |
| `suspicionDelta` | number | ✅ | 怀疑值增量 |
| `required` | boolean | ❌ | 是否必须点击（默认 false） |
| `clickLimit` | number | ❌ | 最大点击次数（默认 1） |

```json
{
  "hotspots": [
    {
      "id": "bloodstain",
      "x": 0.32,
      "y": 0.45,
      "width": 0.08,
      "height": 0.06,
      "description": "地板上有块深色痕迹，摸起来有点粘...",
      "suspicionDelta": 15,
      "required": false,
      "clickLimit": 1
    },
    {
      "id": "knife",
      "x": 0.65,
      "y": 0.52,
      "width": 0.05,
      "height": 0.08,
      "description": "厨房台面有把刀，刀柄上有奇怪的味道...",
      "suspicionDelta": 20,
      "required": false,
      "clickLimit": 1
    }
  ]
}
```

#### 疑点设计规范

- **数量**：每关 5-8 个疑点
- **坐标**：使用 0-1 相对坐标，适配不同分辨率
- **怀疑值**：
  - 明显疑点（血迹、凶器）：15-25
  - 隐蔽疑点（细微不协调）：5-10
  - 无害疑点（装饰品）：0-5
- **点击限制**：避免玩家反复点击刷怀疑值

---

### 5. dialogues（必填）

对话树配置。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `opening` | string | ✅ | NPC 开场白 |
| `rounds` | array | ✅ | 对话轮次数组 |
| `ending` | object | ✅ | 结局台词 |

#### 5.1 rounds 数组

每轮对话包含：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `npc` | string | ✅ | NPC 问题/台词 |
| `options` | array | ✅ | 玩家选项数组 |

#### 5.2 options 数组

每个选项包含：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `text` | string | ✅ | 选项文本 |
| `suspicionDelta` | number | ✅ | 怀疑值变化 |
| `feedback` | string | ✅ | NPC 反馈台词 |
| `next` | number | ✅ | 下一轮索引 |

#### 5.3 ending 对象

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `win` | string | ✅ | 胜利结局台词 |
| `lose` | string | ✅ | 失败结局台词 |

```json
{
  "dialogues": {
    "opening": "你就是新来的钢琴老师？怎么现在才来...",
    "rounds": [
      {
        "npc": "你是怎么找到这份工作的？",
        "options": [
          {
            "text": "中介介绍的",
            "suspicionDelta": 0,
            "feedback": "哦...中介啊。",
            "next": 1
          },
          {
            "text": "你问这么细干嘛？",
            "suspicionDelta": 20,
            "feedback": "没什么，随便问问。",
            "next": 1
          }
        ]
      }
    ],
    "ending": {
      "win": "行了，钢琴你调好了。今天的事...别跟别人说。",
      "lose": "等等...你刚才说的那个细节，不可能是盲人能注意到的。"
    }
  }
}
```

#### 对话设计规范

- **轮数**：每关 5-8 轮对话
- **选项数**：每轮 2-4 个选项
- **怀疑值分布**：
  - 安全选项：0-5
  - 中等风险：10-20
  - 高风险：25-40
- **反馈文本**：NPC 对每个选项的反应，增强沉浸感

---

### 6. rules（必填）

游戏规则配置。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `maxSuspicion` | number | ✅ | 怀疑值上限（默认 100） |
| `failThreshold` | number | ✅ | 失败阈值（默认 60） |
| `minRounds` | number | ✅ | 最少对话轮数（默认 5） |
| `clickPenalty` | boolean | ✅ | 是否启用点击惩罚（默认 true） |

```json
{
  "rules": {
    "maxSuspicion": 100,
    "failThreshold": 60,
    "minRounds": 5,
    "clickPenalty": true
  }
}
```

---

## ✅ 配置校验规则

### 必填字段检查

- `version`、`meta`、`scene`、`hotspots`、`dialogues`、`rules` 必须存在
- `meta.id`、`meta.name` 不能为空
- `hotspots` 数组至少 1 个元素
- `dialogues.rounds` 数组至少 5 个元素

### 数值范围检查

- `hotspots[].x`、`hotspots[].y` 必须在 0-1 之间
- `hotspots[].width`、`hotspots[].height` 必须在 0-0.5 之间
- `suspicionDelta` 必须 >= 0
- `difficulty` 必须在 1-5 之间

### 逻辑检查

- 对话轮次 `next` 索引必须有效
- 最后一轮对话的 `next` 应指向结局
- 疑点 ID 不能重复

---

## 🤖 AI 生成指南

使用 AI 生成关卡配置时：

1. **提供清晰描述**：场景、NPC、玩家身份、核心疑点
2. **指定疑点数量**：5-8 个
3. **指定对话轮数**：5-8 轮
4. **检查输出格式**：确保 JSON 合法
5. **人工校验**：检查逻辑合理性

AI 提示词见：[tools/ai-config-prompt.md](../tools/ai-config-prompt.md)

---

## 📝 配置示例

完整示例见：[src/data/levels/level-01.json](../src/data/levels/level-01.json)

---

*最后更新：2026-03-13*
