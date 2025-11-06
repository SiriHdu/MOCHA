# Media Assets

This folder contains demonstration videos and images for the GHOST Planner project.

## Contents

- `avoidance_event_01.gif` - Multi-robot collision avoidance demo

## Adding More Media

To add your own demos:

1. **Record RViz visualization:**
   ```bash
   # Use screen recording tool (e.g., SimpleScreenRecorder, Kazam)
   # Or record ROS bag and replay
   ```

2. **Convert video to GIF (optional):**
   ```bash
   ffmpeg -i your_video.mp4 -vf "fps=10,scale=800:-1:flags=lanczos" -loop 0 avoidance_event_02.gif
   ```

3. **Add to README.md:**
   ```markdown
   ![Demo](assets/your_new_demo.gif)
   ```

## Recommended Formats

- **GIF**: For short demos (< 10 seconds), good for README
- **MP4**: For longer videos, link to YouTube or host externally
- **PNG**: For static screenshots of RViz, architecture diagrams

## Size Guidelines

- Keep GIF files under 10 MB for fast loading
- Optimize images before committing
- Consider using external hosting (YouTube, Imgur) for large videos

