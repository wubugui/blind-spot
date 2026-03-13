# Vercel 部署指南（血泪版）

> ⚠️ **本文档是用陛下的耐心换来的，每条都是教训！**

---

## 🚀 正确的部署流程

### 方法 1：CLI 手动部署（当前使用）

#### 首次部署

```bash
cd simplegame
vercel --prod
```

这会：
- 创建 `.vercel` 目录（**千万不要删除！**）
- 链接到 Vercel 项目
- 上传文件并部署

#### 后续更新

```bash
# 直接运行，不要删除 .vercel！
vercel --prod
```

**如果遇到问题（比如缓存问题）：**

```bash
# 强制清除缓存重新构建
vercel --prod --force
```

---

### 方法 2：Git 自动部署（推荐⭐⭐⭐）

**这是 Vercel 官方最推荐的方式！**

#### 第一步：初始化 Git

```bash
cd simplegame
git init
git add .
git commit -m "Initial commit"
```

#### 第二步：创建 GitHub 仓库

1. 访问 https://github.com/new
2. 创建仓库（如 `blind-spot`）
3. 复制仓库 URL

#### 第三步：推送代码

```bash
git remote add origin https://github.com/你的用户名/blind-spot.git
git branch -M main
git push -u origin main
```

#### 第四步：关联 Vercel

1. 访问 https://vercel.com/new
2. 导入 GitHub 仓库
3. 点击 Deploy

**完成！之后每次 `git push` 都会自动部署！**

---

## ⚠️ 常见错误与解决方案

### 错误 1：删除了 .vercel 目录

**症状：** 部署到错误的项目或创建新项目

**解决：**
```bash
# 重新链接
vercel link
# 然后正常部署
vercel --prod
```

**预防：** 永远不要手动删除 `.vercel` 目录！

---

### 错误 2：修改代码后部署没变化

**症状：** 运行 `vercel --prod` 但线上还是旧版本

**原因：** Vercel 构建缓存

**解决：**
```bash
# 强制重新构建
vercel --prod --force
```

**预防：** 重要更新时使用 `--force` 参数

---

### 错误 3：浏览器缓存问题

**症状：** 部署成功但刷新后还是旧页面

**原因：** 浏览器缓存了旧文件

**解决：**

1. **强制刷新浏览器**
   - Chrome/Edge: `Ctrl+Shift+R` (Windows) 或 `Cmd+Shift+R` (Mac)
   - Safari: `Cmd+Option+R`
   - 手机：长按刷新按钮 → "清空缓存并重新加载"

2. **访问带版本号的链接**
   ```
   https://simplegame-rosy.vercel.app/?v=2
   ```

3. **使用无痕模式测试**

---

### 错误 4：背景图加载失败

**症状：** 报错 "scene.background 不能为空"

**原因：** 配置中 `background: ""`（空字符串）

**解决：**
```json
// ❌ 错误
"background": ""

// ✅ 正确
"background": null
```

**预防：** 在 `config-loader.js` 中将 background 设为可选字段

---

### 错误 5：资源加载显示 broken image

**症状：** 图片位置显示 broken image 图标

**原因：** 没有处理图片加载失败

**解决：** 添加 onerror 处理

```javascript
// scene.js
this.elements.backgroundImg.onerror = () => {
  this.elements.sceneArea.style.background = 'linear-gradient(...)';
  this.elements.backgroundImg.style.display = 'none';
};
```

---

## 📋 部署检查清单

每次部署前检查：

- [ ] 代码已保存
- [ ] 本地测试通过
- [ ] 配置文件无语法错误
- [ ] 运行 `vercel --prod`
- [ ] 等待部署完成（约 10-30 秒）
- [ ] 获取部署链接
- [ ] 强制刷新浏览器测试
- [ ] 检查控制台无报错

---

## 🔧 常用命令

```bash
# 查看部署状态
vercel ls

# 查看日志
vercel logs

# 强制重新构建
vercel --prod --force

# 部署到 Preview 环境（测试用）
vercel

# 查看帮助
vercel --help
```

---

## 📖 官方文档

- [Vercel 部署概述](https://vercel.com/docs/deployments)
- [Vercel CLI](https://vercel.com/docs/cli)
- [Git 自动部署](https://vercel.com/docs/git)
- [Deploy Hooks](https://vercel.com/docs/deploy-hooks)

---

## 🎯 最佳实践总结

1. **使用 Git 自动部署** - 最稳定可靠
2. **不要删除 .vercel 目录** - 这是项目链接信息
3. **重要更新用 --force** - 避免缓存问题
4. **资源加载加错误处理** - onload/onerror 都要有
5. **部署后强制刷新** - 用版本号或无痕模式测试
6. **配置必填项验证** - 但要区分必填和可选

---

*最后更新：2026-03-13（血泪史）*
