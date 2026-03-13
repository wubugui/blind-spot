/**
 * 状态管理模块
 * 管理游戏状态：怀疑值、当前关卡、进度等
 */

const GameState = {
  /**
   * 当前游戏状态
   */
  state: {
    currentLevelId: null,
    currentLevelConfig: null,
    currentRound: 0,
    suspicion: 0,
    hotspotClicks: {}, // 记录每个疑点的点击次数
    isPlaying: false,
    isEnded: false
  },

  /**
   * 事件监听器
   */
  listeners: {
    suspicionChange: [],
    roundChange: [],
    gameEnd: []
  },

  /**
   * 初始化游戏状态
   * @param {string} levelId - 关卡 ID
   * @param {Object} config - 关卡配置
   */
  init(levelId, config) {
    this.state = {
      currentLevelId: levelId,
      currentLevelConfig: config,
      currentRound: 0,
      suspicion: 0,
      hotspotClicks: {},
      isPlaying: true,
      isEnded: false
    };

    console.log(`[GameState] 游戏初始化：${levelId}`);
    this.notify('suspicionChange', 0);
  },

  /**
   * 获取当前状态
   * @returns {Object}
   */
  getState() {
    return { ...this.state };
  },

  /**
   * 获取当前关卡配置
   * @returns {Object}
   */
  getConfig() {
    return this.state.currentLevelConfig;
  },

  /**
   * 获取当前怀疑值
   * @returns {number}
   */
  getSuspicion() {
    return this.state.suspicion;
  },

  /**
   * 获取当前轮次
   * @returns {number}
   */
  getCurrentRound() {
    return this.state.currentRound;
  },

  /**
   * 增加怀疑值
   * @param {number} delta - 变化值
   * @returns {Object} - { oldValue, newValue, isFailed }
   */
  addSuspicion(delta) {
    const oldValue = this.state.suspicion;
    const newValue = Math.min(oldValue + delta, this.state.currentLevelConfig.rules.maxSuspicion);
    
    this.state.suspicion = newValue;
    
    console.log(`[GameState] 怀疑值变化：${oldValue} -> ${newValue} (${delta >= 0 ? '+' : ''}${delta})`);
    
    this.notify('suspicionChange', newValue);

    // 检查是否失败
    const isFailed = newValue >= this.state.currentLevelConfig.rules.failThreshold;
    
    return {
      oldValue,
      newValue,
      delta,
      isFailed
    };
  },

  /**
   * 推进到下一轮对话
   * @returns {Object} - { round, isLastRound }
   */
  nextRound() {
    const rounds = this.state.currentLevelConfig.dialogues.rounds;
    this.state.currentRound++;
    
    const isLastRound = this.state.currentRound >= rounds.length;
    
    console.log(`[GameState] 进入第 ${this.state.currentRound + 1} 轮`);
    
    this.notify('roundChange', this.state.currentRound);

    return {
      round: this.state.currentRound,
      isLastRound
    };
  },

  /**
   * 记录疑点点击
   * @param {string} hotspotId - 疑点 ID
   * @returns {Object} - { count, exceeded }
   */
  recordHotspotClick(hotspotId) {
    if (!this.state.hotspotClicks[hotspotId]) {
      this.state.hotspotClicks[hotspotId] = 0;
    }
    
    this.state.hotspotClicks[hotspotId]++;
    const count = this.state.hotspotClicks[hotspotId];
    
    // 查找疑点配置
    const hotspot = this.state.currentLevelConfig.hotspots.find(h => h.id === hotspotId);
    const clickLimit = hotspot?.clickLimit || 1;
    const exceeded = count > clickLimit;
    
    console.log(`[GameState] 疑点点击：${hotspotId} (${count}/${clickLimit})`);
    
    return {
      count,
      clickLimit,
      exceeded
    };
  },

  /**
   * 检查游戏是否应该结束
   * @returns {Object} - { shouldEnd, reason, isWin }
   */
  checkEnd() {
    const config = this.state.currentLevelConfig;
    const rounds = config.dialogues.rounds;

    // 检查失败条件
    if (this.state.suspicion >= config.rules.failThreshold) {
      console.log('[GameState] 游戏结束：怀疑值过高');
      return {
        shouldEnd: true,
        reason: 'suspicion',
        isWin: false
      };
    }

    // 检查胜利条件（完成所有对话轮次）
    if (this.state.currentRound >= rounds.length) {
      console.log('[GameState] 游戏结束：完成所有对话');
      return {
        shouldEnd: true,
        reason: 'completed',
        isWin: true
      };
    }

    return {
      shouldEnd: false
    };
  },

  /**
   * 结束游戏
   * @param {boolean} isWin - 是否胜利
   */
  endGame(isWin) {
    this.state.isPlaying = false;
    this.state.isEnded = true;
    
    console.log(`[GameState] 游戏结束：${isWin ? '胜利' : '失败'}`);
    
    this.notify('gameEnd', { isWin, suspicion: this.state.suspicion });
  },

  /**
   * 重置游戏
   */
  reset() {
    if (this.state.currentLevelId && this.state.currentLevelConfig) {
      this.init(this.state.currentLevelId, this.state.currentLevelConfig);
    }
  },

  /**
   * 注册事件监听器
   * @param {string} event - 事件名
   * @param {Function} callback - 回调函数
   */
  on(event, callback) {
    if (this.listeners[event]) {
      this.listeners[event].push(callback);
    }
  },

  /**
   * 移除事件监听器
   * @param {string} event - 事件名
   * @param {Function} callback - 回调函数
   */
  off(event, callback) {
    if (this.listeners[event]) {
      this.listeners[event] = this.listeners[event].filter(cb => cb !== callback);
    }
  },

  /**
   * 通知监听器
   * @param {string} event - 事件名
   * @param {*} data - 数据
   */
  notify(event, data) {
    if (this.listeners[event]) {
      this.listeners[event].forEach(callback => {
        try {
          callback(data);
        } catch (e) {
          console.error(`[GameState] 事件回调错误：${event}`, e);
        }
      });
    }
  },

  /**
   * 保存游戏进度
   */
  save() {
    Utils.storage.set('blindspot_save', {
      levelId: this.state.currentLevelId,
      round: this.state.currentRound,
      suspicion: this.state.suspicion,
      hotspotClicks: this.state.hotspotClicks,
      timestamp: Date.now()
    });
    console.log('[GameState] 游戏进度已保存');
  },

  /**
   * 加载游戏进度
   * @returns {Object|null} - 保存的数据或 null
   */
  load() {
    const save = Utils.storage.get('blindspot_save');
    if (save) {
      console.log('[GameState] 加载游戏进度');
      return save;
    }
    return null;
  },

  /**
   * 清除保存
   */
  clearSave() {
    Utils.storage.remove('blindspot_save');
    console.log('[GameState] 清除保存');
  }
};

// 导出
if (typeof window !== 'undefined') {
  window.GameState = GameState;
}
