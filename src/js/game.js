/**
 * 游戏主循环模块
 * 游戏入口和主控逻辑
 */

const Game = {
  /**
   * 游戏是否已初始化
   */
  initialized: false,

  /**
   * 初始化游戏
   */
  async init() {
    if (this.initialized) {
      console.log('[Game] 游戏已初始化');
      return;
    }

    console.log('[Game] 游戏初始化...');

    // 初始化各模块
    SceneRenderer.init();
    DialogueSystem.init();

    // 注册状态监听
    this.registerListeners();

    // 绑定 UI 事件
    this.bindUIEvents();

    this.initialized = true;
    console.log('[Game] 游戏初始化完成');

    // 检查是否有保存的进度
    const save = GameState.load();
    if (save) {
      console.log('[Game] 发现保存的进度');
      // 可以选择是否询问玩家是否继续
    }
  },

  /**
   * 注册状态监听器
   */
  registerListeners() {
    // 怀疑值变化
    GameState.on('suspicionChange', (value) => {
      this.updateSuspicionUI(value);
    });

    // 轮次变化
    GameState.on('roundChange', (round) => {
      console.log(`[Game] 进入第 ${round + 1} 轮`);
    });

    // 游戏结束
    GameState.on('gameEnd', (result) => {
      this.handleGameEnd(result.isWin);
    });
  },

  /**
   * 绑定 UI 事件
   */
  bindUIEvents() {
    // 重新开始按钮
    document.getElementById('restart-btn')?.addEventListener('click', () => {
      this.restart();
    });

    // 返回菜单按钮
    document.getElementById('menu-btn')?.addEventListener('click', () => {
      this.showMenu();
    });
  },

  /**
   * 更新怀疑值 UI
   * @param {number} value - 当前怀疑值
   */
  updateSuspicionUI(value) {
    const fill = document.getElementById('suspicion-fill');
    const valueText = document.getElementById('suspicion-value');

    if (fill && valueText) {
      fill.style.width = `${value}%`;
      valueText.textContent = `${value}/100`;

      // 根据数值改变颜色
      if (value < 40) {
        fill.style.background = 'linear-gradient(90deg, #4CAF50 0%, #8BC34A 100%)';
      } else if (value < 70) {
        fill.style.background = 'linear-gradient(90deg, #FFC107 0%, #FF9800 100%)';
      } else {
        fill.style.background = 'linear-gradient(90deg, #F44336 0%, #D32F2F 100%)';
      }
    }
  },

  /**
   * 开始新游戏
   * @param {string} levelId - 关卡 ID
   */
  async start(levelId) {
    console.log(`[Game] 开始游戏：${levelId}`);

    try {
      // 加载关卡配置
      const config = await ConfigLoader.loadLevel(levelId);

      // 初始化状态
      GameState.init(levelId, config);

      // 渲染场景
      SceneRenderer.render(config);

      // 显示开场对话
      await DialogueSystem.showOpening(config);

      // 开始第一轮对话
      this.startRound(0);

    } catch (error) {
      console.error('[Game] 游戏启动失败:', error);
      alert(`游戏启动失败：${error.message}`);
    }
  },

  /**
   * 开始对话轮次
   * @param {number} roundIndex - 轮次索引
   */
  async startRound(roundIndex) {
    const config = GameState.getConfig();
    const rounds = config.dialogues.rounds;

    if (roundIndex >= rounds.length) {
      // 所有对话完成，胜利
      GameState.endGame(true);
      return;
    }

    const round = rounds[roundIndex];

    // 显示对话轮次
    await DialogueSystem.showRound(round, (option, index) => {
      this.handleOptionSelect(option, round, index);
    });
  },

  /**
   * 处理选项选择
   * @param {Object} option - 选中的选项
   * @param {Object} round - 当前轮次配置
   * @param {number} index - 选项索引
   */
  async handleOptionSelect(option, round, index) {
    console.log(`[Game] 玩家选择：${option.text} (怀疑值变化：${option.suspicionDelta})`);

    // 更新怀疑值
    if (option.suspicionDelta !== 0) {
      const result = GameState.addSuspicion(option.suspicionDelta);
      
      // 检查是否失败
      if (result.isFailed) {
        await DialogueSystem.showFeedback(option.feedback);
        await DialogueSystem.waitForClick('点击继续...');
        GameState.endGame(false);
        return;
      }
    }

    // 显示 NPC 反馈
    await DialogueSystem.showFeedback(option.feedback);
    
    // 等待玩家点击继续
    await DialogueSystem.waitForClick('点击继续...');

    // 推进到下一轮
    const roundResult = GameState.nextRound();
    
    // 开始下一轮
    await this.startRound(GameState.getCurrentRound());
  },

  /**
   * 处理游戏结束
   * @param {boolean} isWin - 是否胜利
   */
  async handleGameEnd(isWin) {
    console.log(`[Game] 游戏结束：${isWin ? '胜利' : '失败'}`);

    const config = GameState.getConfig();
    
    // 保存进度（如果需要）
    if (!isWin) {
      GameState.clearSave();
    }

    // 显示结局
    await Utils.sleep(500);
    DialogueSystem.showEnding(isWin, config.dialogues.ending);
  },

  /**
   * 重新开始
   */
  async restart() {
    console.log('[Game] 重新开始');
    
    // 隐藏结局画面
    DialogueSystem.hideEnding();
    
    // 重置状态
    GameState.reset();
    
    // 重新开始
    const levelId = GameState.getState().currentLevelId;
    await this.start(levelId);
  },

  /**
   * 返回菜单
   */
  showMenu() {
    console.log('[Game] 返回菜单');
    // TODO: 实现菜单界面
    DialogueSystem.hideEnding();
    GameState.clearSave();
    
    // 显示关卡选择（简单实现：提示输入关卡 ID）
    const levelId = prompt('输入关卡 ID（如 level-01）:');
    if (levelId) {
      this.start(levelId);
    }
  },

  /**
   * 加载指定关卡
   * @param {string} levelId - 关卡 ID
   */
  async loadLevel(levelId) {
    await this.start(levelId);
  }
};

// 导出
if (typeof window !== 'undefined') {
  window.Game = Game;

  // 页面加载完成后初始化
  window.addEventListener('DOMContentLoaded', () => {
    Game.init();

    // 检查 URL 参数是否有关卡 ID
    const levelId = Utils.getUrlParam('level');
    if (levelId) {
      Game.start(levelId);
    } else {
      // 默认加载第一关
      Game.start('level-01');
    }
  });
}
