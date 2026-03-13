/**
 * 场景渲染模块
 * 负责渲染背景、疑点热区、处理点击事件
 */

const SceneRenderer = {
  /**
   * DOM 元素引用
   */
  elements: {
    sceneArea: null,
    backgroundImg: null,
    hotspotsContainer: null,
    tooltip: null
  },

  /**
   * 当前场景数据
   */
  currentScene: null,

  /**
   * 初始化
   */
  init() {
    this.elements.sceneArea = document.getElementById('scene-area');
    this.elements.backgroundImg = document.getElementById('background-img');
    this.elements.hotspotsContainer = document.getElementById('hotspots-container');

    // 创建提示框
    this.elements.tooltip = document.createElement('div');
    this.elements.tooltip.id = 'hotspot-tooltip';
    this.elements.tooltip.className = 'hidden';
    this.elements.sceneArea.appendChild(this.elements.tooltip);

    console.log('[SceneRenderer] 初始化完成');
  },

  /**
   * 渲染场景
   * @param {Object} config - 关卡配置
   */
  render(config) {
    this.currentScene = config;

    // 渲染背景
    this.renderBackground(config.scene.background);

    // 渲染疑点热区
    this.renderHotspots(config.hotspots);

    console.log('[SceneRenderer] 场景渲染完成');
  },

  /**
   * 渲染背景
   * @param {string} backgroundPath - 背景图片路径
   */
  renderBackground(backgroundPath) {
    // 设置默认渐变背景
    this.elements.sceneArea.style.background = 'linear-gradient(135deg, #1a1a2e 0%, #16213e 100%)';
    this.elements.backgroundImg.style.display = 'none';

    // 如果有图片路径，尝试加载
    if (backgroundPath) {
      this.elements.backgroundImg.onload = () => {
        this.elements.backgroundImg.style.display = 'block';
        this.elements.sceneArea.style.background = 'none';
        console.log('[SceneRenderer] 背景图加载成功:', backgroundPath);
      };
      
      this.elements.backgroundImg.onerror = () => {
        console.warn('[SceneRenderer] 背景图加载失败，使用默认背景:', backgroundPath);
        this.elements.backgroundImg.style.display = 'none';
        this.elements.sceneArea.style.background = 'linear-gradient(135deg, #1a1a2e 0%, #16213e 100%)';
      };
      
      // 设置图片源（触发加载）
      this.elements.backgroundImg.src = backgroundPath;
    }
  },

  /**
   * 渲染疑点热区
   * @param {Array} hotspots - 疑点数组
   */
  renderHotspots(hotspots) {
    // 清空现有热区
    this.elements.hotspotsContainer.innerHTML = '';

    hotspots.forEach(hotspot => {
      const element = document.createElement('div');
      element.className = 'hotspot';
      element.dataset.id = hotspot.id;

      // 设置位置（相对坐标转百分比）
      element.style.left = `${hotspot.x * 100}%`;
      element.style.top = `${hotspot.y * 100}%`;
      element.style.width = `${hotspot.width * 100}%`;
      element.style.height = `${hotspot.height * 100}%`;

      // 绑定事件
      element.addEventListener('click', (e) => this.handleHotspotClick(e, hotspot));
      element.addEventListener('mouseenter', (e) => this.handleHotspotHover(e, hotspot));
      element.addEventListener('mouseleave', () => this.handleHotspotLeave());

      this.elements.hotspotsContainer.appendChild(element);
    });

    console.log(`[SceneRenderer] 渲染 ${hotspots.length} 个疑点`);
  },

  /**
   * 处理疑点点击
   * @param {Event} e - 点击事件
   * @param {Object} hotspot - 疑点配置
   */
  handleHotspotClick(e, hotspot) {
    e.stopPropagation();

    const element = e.target;
    
    // 记录点击
    const clickResult = GameState.recordHotspotClick(hotspot.id);

    // 检查是否超过点击限制
    if (clickResult.exceeded) {
      console.log(`[SceneRenderer] 疑点 ${hotspot.id} 已达到点击限制`);
      return;
    }

    // 标记为已点击
    element.classList.add('clicked');

    // 显示描述
    this.showTooltip(hotspot.description);

    // 增加怀疑值（如果配置了惩罚）
    if (this.currentScene.rules.clickPenalty && hotspot.suspicionDelta > 0) {
      const result = GameState.addSuspicion(hotspot.suspicionDelta);
      
      // 检查是否失败
      if (result.isFailed) {
        Game.handleGameEnd(false);
      }
    }

    console.log(`[SceneRenderer] 点击疑点：${hotspot.id}`);
  },

  /**
   * 处理疑点悬停
   * @param {Event} e - 鼠标事件
   * @param {Object} hotspot - 疑点配置
   */
  handleHotspotHover(e, hotspot) {
    // 可以在这里显示提示信息
  },

  /**
   * 处理鼠标离开
   */
  handleHotspotLeave() {
    this.hideTooltip();
  },

  /**
   * 显示提示框
   * @param {string} text - 提示文本
   */
  showTooltip(text) {
    this.elements.tooltip.textContent = text;
    this.elements.tooltip.classList.remove('hidden');

    // 定位到鼠标位置附近
    const rect = this.elements.sceneArea.getBoundingClientRect();
    this.elements.tooltip.style.left = '50%';
    this.elements.tooltip.style.top = '60%';
    this.elements.tooltip.style.transform = 'translate(-50%, -50%)';

    // 自动隐藏
    setTimeout(() => {
      this.hideTooltip();
    }, 3000);
  },

  /**
   * 隐藏提示框
   */
  hideTooltip() {
    this.elements.tooltip.classList.add('hidden');
  },

  /**
   * 清除场景
   */
  clear() {
    this.elements.hotspotsContainer.innerHTML = '';
    this.elements.backgroundImg.src = '';
    this.hideTooltip();
    this.currentScene = null;
  },

  /**
   * 显示/隐藏疑点（用于提示或调试）
   * @param {boolean} show - 是否显示
   */
  toggleHotspots(show) {
    const hotspots = this.elements.hotspotsContainer.querySelectorAll('.hotspot');
    hotspots.forEach(h => {
      h.style.display = show ? 'block' : 'none';
    });
  }
};

// 导出
if (typeof window !== 'undefined') {
  window.SceneRenderer = SceneRenderer;
}
