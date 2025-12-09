import React from 'react';
import Layout from '@theme/Layout';
import styles from './about.module.css';

function About() {
  return (
    <Layout title="About" description="About the author and the Physical AI & Humanoid Robotics Book">
      
      <section className={styles.aboutSection}>
        <div className="container">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className={styles.aboutCard}>
                <h2 className={styles.sectionTitle}>About the Physical AI & Humanoid Robotics Book</h2>
                <p className={styles.aboutText}>
                  <strong>Aqsa Gull</strong> is a specialized researcher and developer in the field of
                  Physical AI and Humanoid Robotics. With expertise spanning AI integration,
                  robotics control systems, and advanced robotic platforms including ROS 2 and
                  NVIDIA Isaac ecosystems.
                </p>
                <p className={styles.aboutText}>
                  This comprehensive guide represents years of research and practical experience
                  in building intelligent humanoid robots, combining theoretical foundations
                  with hands-on implementation strategies.
                </p>
                <div className={styles.featuresGrid}>
                  <div className={styles.featureItem}>
                    <h3>ROS 2 & Robotic Nervous System</h3>
                    <p>Learn how to build the communication backbone for your humanoid robot with ROS 2 architecture.</p>
                  </div>
                  <div className={styles.featureItem}>
                    <h3>Digital Twin & Simulation</h3>
                    <p>Master Gazebo and Unity for high-fidelity physics simulation and sensor modeling.</p>
                  </div>
                  <div className={styles.featureItem}>
                    <h3>Vision-Language-Action Systems</h3>
                    <p>Integrate LLMs and perception systems for intelligent robot behavior.</p>
                  </div>
                </div>
                <p className={styles.aboutText}>
                  The book aims to bridge the gap between academic research and practical
                  implementation in the rapidly evolving field of humanoid robotics and
                  physical AI systems.
                </p>
              </div>
            </div>
          </div>
        </div>
      </section>
    </Layout>
  );
}

export default About;



