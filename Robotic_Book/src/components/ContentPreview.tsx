import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import { motion } from 'framer-motion';
import clsx from 'clsx';

import styles from './ContentPreview.module.css';

interface ContentCardProps {
  title: string;
  description: string;
  delay: number;
}

function ContentCard({ title, description, delay }: ContentCardProps): ReactNode {
  return (
    <motion.div
      className="col col--4"
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.6, delay: delay }}
      whileHover={{ y: -10 }}
    >
      <div className={styles.previewCard}>
        <Heading as="h3" className={styles.cardTitle}>
          {title}
        </Heading>
        <p className={styles.cardDescription}>{description}</p>
      </div>
    </motion.div>
  );
}

export default function ContentPreview(): ReactNode {
  return (
    <motion.section
      className={clsx(styles.contentPreview, 'margin-vert--lg')}
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      transition={{ duration: 0.8, delay: 0.3 }}
    >
      <div className="container">
        <div className="row">
          <ContentCard
            title="ROS 2 & Robotic Nervous System"
            description="Learn how to build the communication backbone for your humanoid robot with ROS 2 architecture."
            delay={0.2}
          />
          <ContentCard
            title="Digital Twin & Simulation"
            description="Master Gazebo and Unity for high-fidelity physics simulation and sensor modeling."
            delay={0.4}
          />
          <ContentCard
            title="Vision-Language-Action Systems"
            description="Integrate LLMs and perception systems for intelligent robot behavior."
            delay={0.6}
          />
        </div>
      </div>
    </motion.section>
  );
}