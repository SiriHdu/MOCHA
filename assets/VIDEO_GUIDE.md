# 视频展示指南

本指南教你如何在README中添加演示视频。

## 📋 快速选择

| 方法 | 适用场景 | 优点 | 缺点 |
|------|---------|------|------|
| **GIF** | 短片段(<10秒) | 自动播放，兼容性好 | 文件大，画质一般 |
| **MP4直接嵌入** | 中等长度(<50MB) | 画质好，控制播放 | GitHub限制100MB |
| **YouTube链接** | 长视频(>1分钟) | 无大小限制，专业 | 需要YouTube账号 |

---

## 方法1️⃣: 转换视频为GIF（当前使用）

### 安装FFmpeg
```bash
sudo apt install ffmpeg
```

### 转换命令
```bash
# 基础转换（800px宽，10帧/秒）
ffmpeg -i your_video.mp4 \
  -vf "fps=10,scale=800:-1:flags=lanczos" \
  -loop 0 \
  assets/demo.gif

# 高质量转换（使用调色板）
ffmpeg -i your_video.mp4 \
  -vf "fps=10,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
  -loop 0 \
  assets/demo_hq.gif

# 压缩版本（小文件）
ffmpeg -i your_video.mp4 \
  -vf "fps=8,scale=640:-1" \
  -loop 0 \
  assets/demo_small.gif
```

### 在README中使用
```markdown
![Demo](assets/demo.gif)

# 或带说明文字
<div align="center">
  <img src="assets/demo.gif" alt="Demo Description" width="800"/>
  <p><i>Real-time multi-robot planning</i></p>
</div>
```

### 优化技巧
- **减小尺寸**: `scale=640:-1` 或 `scale=480:-1`
- **降低帧率**: `fps=8` 或 `fps=6`
- **裁剪时间**: `-ss 00:00:05 -t 10` (从5秒开始，持续10秒)
- **压缩工具**: 使用 https://ezgif.com/optimize 在线压缩

---

## 方法2️⃣: GitHub原生MP4支持

### 步骤1: 准备视频
```bash
# 压缩视频到合理大小（<50MB推荐）
ffmpeg -i your_large_video.mp4 \
  -vcodec libx264 \
  -crf 28 \
  -preset fast \
  assets/demo_compressed.mp4

# 检查文件大小
ls -lh assets/demo_compressed.mp4
```

### 步骤2: 添加到assets
```bash
cp your_video.mp4 ~/ghost/assets/demo_video.mp4
```

### 步骤3: 在README中引用

**简单方式（自动播放）：**
```markdown
## Demo

https://github.com/your-username/ghost/assets/demo_video.mp4
```

**HTML5方式（带控制条）：**
```markdown
<video width="800" controls>
  <source src="assets/demo_video.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
```

---

## 方法3️⃣: YouTube嵌入（推荐长视频）

### 步骤1: 上传到YouTube
1. 访问 https://youtube.com
2. 点击右上角"创建" → "上传视频"
3. 上传你的演示视频
4. 设置为"公开"或"不公开"

### 步骤2: 获取视频ID
YouTube链接格式：`https://www.youtube.com/watch?v=dQw4w9WgXcQ`

视频ID就是 `v=` 后面的部分：**dQw4w9WgXcQ**

### 步骤3: 在README中添加

**基础版：**
```markdown
[![Watch Demo](https://img.youtube.com/vi/YOUR_VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)
```

**高清预览图：**
```markdown
[![Watch Demo](https://img.youtube.com/vi/YOUR_VIDEO_ID/maxresdefault.jpg)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)
```

**带说明文字：**
```markdown
<div align="center">

[![GHOST Planner Demo](https://img.youtube.com/vi/YOUR_VIDEO_ID/maxresdefault.jpg)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)

**🎬 Click to watch full demonstration on YouTube**

*Multi-robot collision avoidance in complex environments*

</div>
```

**替换示例：**
```markdown
# 把YOUR_VIDEO_ID替换成实际ID
[![Demo](https://img.youtube.com/vi/dQw4w9WgXcQ/maxresdefault.jpg)](https://www.youtube.com/watch?v=dQw4w9WgXcQ)
```

---

## 方法4️⃣: 多媒体组合展示

### 布局1: 并排两个视频
```markdown
<table>
  <tr>
    <td><img src="assets/demo1.gif" width="400"/></td>
    <td><img src="assets/demo2.gif" width="400"/></td>
  </tr>
  <tr>
    <td align="center"><i>Scenario A</i></td>
    <td align="center"><i>Scenario B</i></td>
  </tr>
</table>
```

### 布局2: GIF + YouTube
```markdown
## Demonstrations

### Quick Preview (GIF)
![Quick Demo](assets/quick_demo.gif)

### Full Video (YouTube)
[![Full Demo](https://img.youtube.com/vi/YOUR_ID/maxresdefault.jpg)](https://www.youtube.com/watch?v=YOUR_ID)
```

### 布局3: 多场景展示
```markdown
## Demo Gallery

<div align="center">

| Collision Avoidance | Path Planning | Multi-Robot |
|:---:|:---:|:---:|
| ![](assets/avoid.gif) | ![](assets/plan.gif) | ![](assets/multi.gif) |
| Dynamic obstacle avoidance | Homotopy path generation | Coordinated navigation |

</div>
```

---

## 录制视频建议

### 工具推荐

**Ubuntu屏幕录制：**
```bash
# SimpleScreenRecorder（推荐）
sudo apt install simplescreenrecorder

# Kazam
sudo apt install kazam

# OBS Studio（专业级）
sudo apt install obs-studio
```

**ROS2 bag录制（推荐）：**
```bash
# 录制所有话题
ros2 bag record -a -o demo_recording

# 播放并录制屏幕
ros2 bag play demo_recording.db3
# 同时用录屏软件录制RViz
```

### 录制技巧

✅ **分辨率**: 1920x1080 或 1280x720  
✅ **帧率**: 30fps（转GIF时会降低）  
✅ **时长**: 
   - GIF: 5-15秒
   - MP4: 30-120秒
   - YouTube: 无限制  
✅ **内容**:
   - 先展示初始场景 (2秒)
   - 演示主要功能 (10-30秒)
   - 展示结果 (2秒)

---

## 实际操作示例

### 场景：你有一个30秒的MP4视频

```bash
cd ~/ghost/assets

# 1. 创建短GIF（首页展示）
ffmpeg -i full_demo.mp4 -ss 00:00:05 -t 8 \
  -vf "fps=10,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
  demo_short.gif

# 2. 压缩MP4（GitHub嵌入）
ffmpeg -i full_demo.mp4 \
  -vcodec libx264 -crf 28 -preset fast \
  demo_compressed.mp4

# 3. 上传完整版到YouTube（长视频）
# 手动上传 full_demo.mp4
```

**README中使用：**
```markdown
## Demo

<!-- 顶部快速预览 -->
![Quick Preview](assets/demo_short.gif)

<!-- 完整演示视频 -->
### Full Demonstration

**Embedded Video:**
https://github.com/username/ghost/assets/demo_compressed.mp4

**YouTube HD Version:**
[![Watch on YouTube](https://img.youtube.com/vi/YOUR_ID/maxresdefault.jpg)](https://www.youtube.com/watch?v=YOUR_ID)
```

---

## 常见问题

### Q: GIF太大怎么办？
```bash
# 方法1: 降低分辨率和帧率
ffmpeg -i input.mp4 -vf "fps=6,scale=480:-1" output.gif

# 方法2: 使用在线工具压缩
# https://ezgif.com/optimize

# 方法3: 只保留关键片段
ffmpeg -i input.mp4 -ss 00:00:10 -t 5 output.gif
```

### Q: GitHub不显示视频？
- 确保文件路径正确: `assets/video.mp4`
- 文件大小 < 100MB
- 推送到GitHub后等待几分钟处理
- 使用相对路径，不要用绝对路径

### Q: YouTube视频预览图不清晰？
```markdown
# 使用高清预览（maxresdefault）
https://img.youtube.com/vi/YOUR_ID/maxresdefault.jpg

# 如果不存在，降级使用
https://img.youtube.com/vi/YOUR_ID/hqdefault.jpg
```

### Q: 想要自动播放？
```markdown
<!-- GIF会自动播放 -->
![Auto Play](assets/demo.gif)

<!-- MP4需要用户点击播放 -->
https://github.com/user/repo/assets/demo.mp4
```

---

## 检查清单

提交前确认：

- [ ] 视频文件在 `assets/` 目录
- [ ] 文件大小合理（GIF<10MB, MP4<50MB）
- [ ] README中路径正确
- [ ] 本地预览显示正常
- [ ] 添加了说明文字
- [ ] .gitignore 不会忽略视频文件

---

## 推荐配置（当前项目）

根据GHOST Planner的特点，推荐：

**首页（README顶部）：**
- 1个精彩的8-10秒GIF（800px宽）
- 展示核心功能：多机器人避障

**Demo章节：**
- 2-3个不同场景的GIF（并排或网格）
- 链接到YouTube完整演示（可选）

**执行：**
```bash
# 假设你有多个演示视频
cd ~/ghost/assets

# 主GIF（首页）
ffmpeg -i main_demo.mp4 -ss 5 -t 10 \
  -vf "fps=10,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
  avoidance_event_01.gif

# 场景2
ffmpeg -i scenario2.mp4 -ss 3 -t 8 \
  -vf "fps=10,scale=600:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
  planning_demo.gif

# 场景3
ffmpeg -i scenario3.mp4 -ss 2 -t 8 \
  -vf "fps=10,scale=600:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
  multi_robot_demo.gif
```

搞定！🎬

