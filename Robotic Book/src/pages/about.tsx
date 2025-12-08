import React from 'react';
import Layout from '@theme/Layout';

function About() {
  return (
    <Layout title="About" description="About the author and the Physical AI & Humanoid Robotics Book">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>About the Author</h1>
            <p>
              <strong>Aqsa Gull</strong> is a specialized researcher and developer in the field of
              Physical AI and Humanoid Robotics. With expertise spanning AI integration,
              robotics control systems, and advanced robotic platforms including ROS 2 and
              NVIDIA Isaac ecosystems.
            </p>
            <p>
              This comprehensive guide represents years of research and practical experience
              in building intelligent humanoid robots, combining theoretical foundations
              with hands-on implementation strategies.
            </p>
            <p>
              The book aims to bridge the gap between academic research and practical
              implementation in the rapidly evolving field of humanoid robotics and
              physical AI systems.
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default About;