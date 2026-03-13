/**
 * 配置加载器模块
 * 负责加载和验证关卡 JSON 配置
 */

const ConfigLoader = {
  /**
   * 配置缓存
   */
  cache: new Map(),

  /**
   * 加载关卡配置
   * @param {string} levelId - 关卡 ID
   * @returns {Promise<Object>} - 关卡配置对象
   */
  async loadLevel(levelId) {
    // 检查缓存
    if (this.cache.has(levelId)) {
      console.log(`[ConfigLoader] 从缓存加载关卡：${levelId}`);
      return this.cache.get(levelId);
    }

    // 加载 JSON 文件
    const url = `data/levels/${levelId}.json`;
    console.log(`[ConfigLoader] 加载关卡：${url}`);

    try {
      const response = await fetch(url);
      
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const config = await response.json();
      
      // 验证配置
      this.validate(config);
      
      // 缓存配置
      this.cache.set(levelId, config);
      
      console.log(`[ConfigLoader] 关卡加载成功：${levelId}`);
      return config;
    } catch (error) {
      console.error(`[ConfigLoader] 关卡加载失败：${levelId}`, error);
      throw error;
    }
  },

  /**
   * 从原始数据加载配置（用于 AI 生成或动态创建）
   * @param {Object} configData - 配置数据对象
   * @returns {Object} - 验证后的配置对象
   */
  loadFromData(configData) {
    this.validate(configData);
    return configData;
  },

  /**
   * 验证关卡配置
   * @param {Object} config - 配置对象
   * @throws {Error} - 验证失败时抛出错误
   */
  validate(config) {
    const errors = [];

    // 检查必填字段
    if (!config.version) {
      errors.push('缺少必填字段：version');
    }

    if (!config.meta) {
      errors.push('缺少必填字段：meta');
    } else {
      if (!config.meta.id) errors.push('meta.id 不能为空');
      if (!config.meta.name) errors.push('meta.name 不能为空');
    }

    if (!config.scene) {
      errors.push('缺少必填字段：scene');
    } else {
      // background 改为可选（允许占位背景）
      if (!config.scene.npc) errors.push('scene.npc 不能为空');
      if (!config.scene.player) errors.push('scene.player 不能为空');
    }

    if (!config.hotspots || !Array.isArray(config.hotspots)) {
      errors.push('hotspots 必须是数组');
    } else if (config.hotspots.length === 0) {
      errors.push('hotspots 至少需要 1 个疑点');
    } else {
      // 验证每个疑点
      config.hotspots.forEach((hotspot, index) => {
        if (!hotspot.id) errors.push(`hotspots[${index}].id 不能为空`);
        if (typeof hotspot.x !== 'number' || hotspot.x < 0 || hotspot.x > 1) {
          errors.push(`hotspots[${index}].x 必须在 0-1 之间`);
        }
        if (typeof hotspot.y !== 'number' || hotspot.y < 0 || hotspot.y > 1) {
          errors.push(`hotspots[${index}].y 必须在 0-1 之间`);
        }
        if (typeof hotspot.suspicionDelta !== 'number' || hotspot.suspicionDelta < 0) {
          errors.push(`hotspots[${index}].suspicionDelta 必须 >= 0`);
        }
      });
    }

    if (!config.dialogues) {
      errors.push('缺少必填字段：dialogues');
    } else {
      if (typeof config.dialogues.opening !== 'string') {
        errors.push('dialogues.opening 必须是字符串');
      }
      if (!config.dialogues.rounds || !Array.isArray(config.dialogues.rounds)) {
        errors.push('dialogues.rounds 必须是数组');
      } else if (config.dialogues.rounds.length < 5) {
        errors.push('dialogues.rounds 至少需要 5 轮对话');
      } else {
        // 验证对话轮次
        config.dialogues.rounds.forEach((round, index) => {
          if (!round.npc) errors.push(`rounds[${index}].npc 不能为空`);
          if (!round.options || !Array.isArray(round.options)) {
            errors.push(`rounds[${index}].options 必须是数组`);
          }
        });
      }
      if (!config.dialogues.ending) {
        errors.push('dialogues.ending 不能为空');
      }
    }

    if (!config.rules) {
      errors.push('缺少必填字段：rules');
    } else {
      if (typeof config.rules.maxSuspicion !== 'number') {
        errors.push('rules.maxSuspicion 必须是数字');
      }
      if (typeof config.rules.failThreshold !== 'number') {
        errors.push('rules.failThreshold 必须是数字');
      }
    }

    // 如果有错误，抛出
    if (errors.length > 0) {
      throw new Error(`配置验证失败:\n${errors.join('\n')}`);
    }

    console.log('[ConfigLoader] 配置验证通过');
  },

  /**
   * 获取所有可用关卡列表
   * @returns {Promise<Array>} - 关卡 ID 列表
   */
  async getLevelList() {
    // 开发模式：返回缓存中的所有关卡
    const levels = Array.from(this.cache.keys());
    
    // TODO: 生产模式可以扫描目录或从配置文件读取
    return levels;
  },

  /**
   * 清除缓存
   * @param {string} levelId - 可选，指定清除的关卡 ID
   */
  clearCache(levelId = null) {
    if (levelId) {
      this.cache.delete(levelId);
      console.log(`[ConfigLoader] 清除缓存：${levelId}`);
    } else {
      this.cache.clear();
      console.log('[ConfigLoader] 清除所有缓存');
    }
  },

  /**
   * 热加载配置（开发模式）
   * @param {string} levelId - 关卡 ID
   */
  async hotReload(levelId) {
    this.clearCache(levelId);
    return await this.loadLevel(levelId);
  }
};

// 开发模式：监听配置文件变化
if (typeof window !== 'undefined' && window.location.hostname === 'localhost') {
  // 简单轮询检查（实际开发可以用 WebSocket）
  setInterval(async () => {
    // TODO: 实现文件变化检测
  }, 5000);
}

// 导出
if (typeof window !== 'undefined') {
  window.ConfigLoader = ConfigLoader;
}
