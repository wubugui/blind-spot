#!/usr/bin/env node

/**
 * 关卡配置验证工具
 * 用法：node tools/validate-level.js [levelId]
 */

const fs = require('fs');
const path = require('path');

const levelsDir = path.join(__dirname, '..', 'src', 'data', 'levels');

/**
 * 验证单个关卡配置
 */
function validateLevel(filePath) {
  const fileName = path.basename(filePath);
  console.log(`\n验证：${fileName}`);
  console.log('='.repeat(50));

  try {
    // 读取文件
    const content = fs.readFileSync(filePath, 'utf-8');
    const config = JSON.parse(content);

    // 验证必填字段
    const requiredFields = ['version', 'meta', 'scene', 'hotspots', 'dialogues', 'rules'];
    const missingFields = requiredFields.filter(f => !config[f]);
    
    if (missingFields.length > 0) {
      console.error(`❌ 缺少必填字段：${missingFields.join(', ')}`);
      return false;
    }

    // 验证 meta
    if (!config.meta.id || !config.meta.name) {
      console.error('❌ meta.id 或 meta.name 缺失');
      return false;
    }
    console.log(`✓ 关卡 ID: ${config.meta.id}`);
    console.log(`✓ 关卡名称：${config.meta.name}`);

    // 验证 hotspots
    if (!Array.isArray(config.hotspots) || config.hotspots.length === 0) {
      console.error('❌ hotspots 必须是至少包含 1 个元素的数组');
      return false;
    }
    
    let totalSuspicion = 0;
    config.hotspots.forEach((h, i) => {
      if (h.x < 0 || h.x > 1 || h.y < 0 || h.y > 1) {
        console.error(`❌ hotspot[${i}] 坐标超出范围 (0-1)`);
        return false;
      }
      totalSuspicion += h.suspicionDelta || 0;
    });
    console.log(`✓ 疑点数量：${config.hotspots.length}`);
    console.log(`✓ 疑点总怀疑值：${totalSuspicion}`);

    // 验证 dialogues
    if (!config.dialogues.rounds || !Array.isArray(config.dialogues.rounds)) {
      console.error('❌ dialogues.rounds 必须是数组');
      return false;
    }
    
    if (config.dialogues.rounds.length < 5) {
      console.warn(`⚠ 对话轮数少于 5 轮（当前：${config.dialogues.rounds.length}）`);
    }
    
    let totalOptions = 0;
    config.dialogues.rounds.forEach((round, i) => {
      if (!round.options || !Array.isArray(round.options)) {
        console.error(`❌ rounds[${i}].options 必须是数组`);
        return false;
      }
      if (round.options.length < 2) {
        console.warn(`⚠ rounds[${i}] 选项少于 2 个`);
      }
      totalOptions += round.options.length;
    });
    
    console.log(`✓ 对话轮数：${config.dialogues.rounds.length}`);
    console.log(`✓ 总选项数：${totalOptions}`);

    // 验证 rules
    const failThreshold = config.rules.failThreshold || 60;
    console.log(`✓ 失败阈值：${failThreshold}/100`);

    // 评估难度
    const avgSuspicionPerRound = totalSuspicion / config.dialogues.rounds.length;
    console.log(`✓ 平均每轮疑点怀疑值：${avgSuspicionPerRound.toFixed(1)}`);

    if (avgSuspicionPerRound > 30) {
      console.warn('⚠ 平均怀疑值偏高，可能导致难度过大');
    }

    console.log('\n✅ 验证通过！');
    return true;

  } catch (error) {
    console.error(`❌ 验证失败：${error.message}`);
    return false;
  }
}

/**
 * 验证所有关卡
 */
function validateAll() {
  console.log('盲点 - 关卡配置验证工具');
  console.log('='.repeat(50));

  if (!fs.existsSync(levelsDir)) {
    console.error(`❌ 关卡目录不存在：${levelsDir}`);
    process.exit(1);
  }

  const files = fs.readdirSync(levelsDir).filter(f => f.endsWith('.json'));
  
  if (files.length === 0) {
    console.log('⚠ 未找到关卡配置文件');
    process.exit(0);
  }

  console.log(`找到 ${files.length} 个关卡配置文件\n`);

  let passed = 0;
  let failed = 0;

  files.forEach(file => {
    const filePath = path.join(levelsDir, file);
    if (validateLevel(filePath)) {
      passed++;
    } else {
      failed++;
    }
  });

  console.log('\n' + '='.repeat(50));
  console.log(`验证完成：${passed} 通过，${failed} 失败`);

  if (failed > 0) {
    process.exit(1);
  }
}

// 主程序
const levelId = process.argv[2];

if (levelId) {
  // 验证指定关卡
  const filePath = path.join(levelsDir, `${levelId}.json`);
  if (!fs.existsSync(filePath)) {
    console.error(`❌ 文件不存在：${filePath}`);
    process.exit(1);
  }
  validateLevel(filePath) ? process.exit(0) : process.exit(1);
} else {
  // 验证所有关卡
  validateAll();
}
