/**
 * 工具函数模块
 */

const Utils = {
  /**
   * 打字机效果显示文本
   * @param {HTMLElement} element - 显示文本的元素
   * @param {string} text - 要显示的文本
   * @param {number} speed - 每个字符的间隔（毫秒）
   * @returns {Promise} - 文本显示完成的 Promise
   */
  typeText(element, text, speed = 30) {
    return new Promise((resolve) => {
      element.textContent = '';
      let index = 0;

      const type = () => {
        if (index < text.length) {
          element.textContent += text[index];
          index++;
          setTimeout(type, speed);
        } else {
          resolve();
        }
      };

      type();
    });
  },

  /**
   * 立即显示文本（取消打字机效果）
   * @param {HTMLElement} element - 显示文本的元素
   * @param {string} text - 要显示的文本
   */
  showText(element, text) {
    element.textContent = text;
  },

  /**
   * 淡入效果
   * @param {HTMLElement} element - 要淡入的元素
   * @param {number} duration - 动画时长（毫秒）
   */
  fadeIn(element, duration = 300) {
    element.style.opacity = '0';
    element.style.transition = `opacity ${duration}ms ease`;
    
    setTimeout(() => {
      element.style.opacity = '1';
    }, 50);
  },

  /**
   * 淡出效果
   * @param {HTMLElement} element - 要淡出的元素
   * @param {number} duration - 动画时长（毫秒）
   * @returns {Promise} - 动画完成的 Promise
   */
  fadeOut(element, duration = 300) {
    return new Promise((resolve) => {
      element.style.transition = `opacity ${duration}ms ease`;
      element.style.opacity = '0';
      
      setTimeout(() => {
        resolve();
      }, duration);
    });
  },

  /**
   * 休眠指定时间
   * @param {number} ms - 毫秒数
   * @returns {Promise}
   */
  sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  },

  /**
   * 格式化数字（补零）
   * @param {number} num - 数字
   * @param {number} length - 目标长度
   * @returns {string}
   */
  padNumber(num, length = 2) {
    return String(num).padStart(length, '0');
  },

  /**
   * 获取 URL 参数
   * @param {string} name - 参数名
   * @returns {string|null}
   */
  getUrlParam(name) {
    const params = new URLSearchParams(window.location.search);
    return params.get(name);
  },

  /**
   * 本地存储操作
   */
  storage: {
    get(key, defaultValue = null) {
      try {
        const value = localStorage.getItem(key);
        return value ? JSON.parse(value) : defaultValue;
      } catch (e) {
        return defaultValue;
      }
    },

    set(key, value) {
      try {
        localStorage.setItem(key, JSON.stringify(value));
      } catch (e) {
        console.warn('LocalStorage 写入失败:', e);
      }
    },

    remove(key) {
      try {
        localStorage.removeItem(key);
      } catch (e) {
        console.warn('LocalStorage 删除失败:', e);
      }
    }
  }
};

// 导出（浏览器环境直接挂载到 window）
if (typeof window !== 'undefined') {
  window.Utils = Utils;
}
