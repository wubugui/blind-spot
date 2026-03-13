/**
 * 对话系统模块
 * 负责显示对话、渲染选项、处理玩家选择
 */

const DialogueSystem = {
  /**
   * DOM 元素引用
   */
  elements: {
    npcAvatar: null,
    npcName: null,
    npcText: null,
    optionsContainer: null
  },

  /**
   * 初始化
   */
  init() {
    this.elements.npcAvatar = document.getElementById('npc-avatar');
    this.elements.npcName = document.getElementById('npc-name');
    this.elements.npcText = document.getElementById('npc-text');
    this.elements.optionsContainer = document.getElementById('options-container');

    console.log('[DialogueSystem] 初始化完成');
  },

  /**
   * 渲染 NPC 信息
   * @param {Object} npc - NPC 配置
   */
  renderNpc(npc) {
    // 默认隐藏头像
    this.elements.npcAvatar.style.display = 'none';

    // 如果有头像路径，尝试加载
    if (npc.avatar) {
      this.elements.npcAvatar.onload = () => {
        this.elements.npcAvatar.style.display = 'block';
        console.log('[DialogueSystem] NPC 头像加载成功:', npc.avatar);
      };
      
      this.elements.npcAvatar.onerror = () => {
        console.warn('[DialogueSystem] NPC 头像加载失败，隐藏显示:', npc.avatar);
        this.elements.npcAvatar.style.display = 'none';
      };
      
      // 设置图片源（触发加载）
      this.elements.npcAvatar.src = npc.avatar;
    }

    this.elements.npcName.textContent = npc.name;
  },

  /**
   * 显示 NPC 对话
   * @param {string} text - 对话文本
   * @param {boolean} typeEffect - 是否使用打字机效果
   * @returns {Promise}
   */
  async showNpcText(text, typeEffect = true) {
    if (typeEffect) {
      // 恢复原始速度 30ms 每个字
      await Utils.typeText(this.elements.npcText, text, 30);
    } else {
      Utils.showText(this.elements.npcText, text);
    }
  },

  /**
   * 渲染对话选项
   * @param {Array} options - 选项数组
   * @param {Function} onSelect - 选择回调
   */
  renderOptions(options, onSelect) {
    // 清空现有选项
    this.elements.optionsContainer.innerHTML = '';

    // 创建选项按钮
    options.forEach((option, index) => {
      const button = document.createElement('button');
      button.className = 'option-btn';
      button.textContent = option.text;
      button.dataset.index = index;

      // 绑定点击事件
      button.addEventListener('click', () => {
        // 禁用所有按钮防止重复点击
        this.disableOptions();
        
        // 调用回调
        onSelect(option, index);
      });

      this.elements.optionsContainer.appendChild(button);
    });

    console.log(`[DialogueSystem] 渲染 ${options.length} 个选项`);
  },

  /**
   * 禁用所有选项
   */
  disableOptions() {
    const buttons = this.elements.optionsContainer.querySelectorAll('.option-btn');
    buttons.forEach(btn => {
      btn.disabled = true;
    });
  },

  /**
   * 启用所有选项
   */
  enableOptions() {
    const buttons = this.elements.optionsContainer.querySelectorAll('.option-btn');
    buttons.forEach(btn => {
      btn.disabled = false;
    });
  },

  /**
   * 显示开场对话
   * @param {Object} config - 关卡配置
   */
  async showOpening(config) {
    // 渲染 NPC
    this.renderNpc(config.scene.npc);

    // 显示开场白
    await this.showNpcText(config.dialogues.opening, true);

    // 等待玩家点击对话框继续
    await this.waitForClick('');

    console.log('[DialogueSystem] 开场对话完成');
  },

  /**
   * 显示对话轮次
   * @param {Object} round - 对话轮次配置
   * @param {Function} onSelect - 选择回调
   */
  async showRound(round, onSelect) {
    // 显示 NPC 问题（打字机效果）
    await this.showNpcText(round.npc, true);
    
    // 等待玩家点击继续
    await this.waitForClick('点击继续...');

    // 渲染选项
    this.renderOptions(round.options, onSelect);
  },

  /**
   * 等待玩家点击对话框继续
   * @param {string} promptText - 提示文本（可选）
   * @returns {Promise}
   */
  waitForClick(promptText) {
    return new Promise((resolve) => {
      const dialogueBox = document.getElementById('dialogue-box');
      
      // 设置对话框为可点击
      dialogueBox.style.cursor = 'pointer';
      dialogueBox.title = '点击继续';
      
      // 添加提示文本（如果有）
      let hintElement = null;
      if (promptText) {
        hintElement = document.createElement('span');
        hintElement.className = 'continue-hint';
        hintElement.textContent = '  ' + promptText;
        hintElement.style.cssText = `
          color: #FFC107;
          font-size: 12px;
          animation: pulse 1.5s infinite;
        `;
        dialogueBox.appendChild(hintElement);
      }
      
      // 添加点击事件
      const clickHandler = () => {
        dialogueBox.style.cursor = 'default';
        dialogueBox.title = '';
        dialogueBox.removeEventListener('click', clickHandler);
        if (hintElement) hintElement.remove();
        resolve();
      };
      
      dialogueBox.addEventListener('click', clickHandler);
      
      // 添加动画样式
      if (!document.getElementById('continue-animation-style')) {
        const style = document.createElement('style');
        style.id = 'continue-animation-style';
        style.textContent = `
          @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
          }
          #dialogue-box {
            transition: all 0.2s ease;
          }
          #dialogue-box:hover {
            background: rgba(0, 0, 0, 0.6);
          }
        `;
        document.head.appendChild(style);
      }
    });
  },

  /**
   * 显示 NPC 反馈
   * @param {string} feedback - 反馈文本
   */
  async showFeedback(feedback) {
    await this.showNpcText(feedback, true);
  },

  /**
   * 显示结局
   * @param {boolean} isWin - 是否胜利
   * @param {Object} ending - 结局配置
   */
  showEnding(isWin, ending) {
    const endingScreen = document.getElementById('ending-screen');
    const endingTitle = document.getElementById('ending-title');
    const endingText = document.getElementById('ending-text');

    // 设置标题
    endingTitle.textContent = isWin ? '胜利' : '失败';
    endingTitle.className = isWin ? 'win' : 'lose';

    // 设置文本
    const text = isWin ? ending.win : ending.lose;
    Utils.showText(endingText, text);

    // 显示结局画面
    endingScreen.classList.remove('hidden');

    console.log(`[DialogueSystem] 结局显示：${isWin ? '胜利' : '失败'}`);
  },

  /**
   * 隐藏结局画面
   */
  hideEnding() {
    const endingScreen = document.getElementById('ending-screen');
    endingScreen.classList.add('hidden');
  },

  /**
   * 清除对话
   */
  clear() {
    this.elements.npcText.textContent = '';
    this.elements.optionsContainer.innerHTML = '';
  }
};

// 导出
if (typeof window !== 'undefined') {
  window.DialogueSystem = DialogueSystem;
}
