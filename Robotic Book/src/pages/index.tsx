import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import { motion } from 'framer-motion';
import ContentPreview from '@site/src/components/ContentPreview';

import styles from './index.module.css';
import Footer from '../components/Footer';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroBackgroundImage} />

        <motion.div
          initial={{ opacity: 0, y: -20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          className={styles.heroContent}
        >
          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
        </motion.div>

        <div className={styles.buttons}>
          
          <Link
            to="/docs/intro"
            className={clsx('button button--lg', styles.readyBookButton)}
          >
            Ready Book
          </Link>
          <img
    src="/img/home_image.png" // small-medium icon
    alt="Book Icon"
    className={styles.buttonBranderIconRight}
  />
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout>
      <HomepageHeader />
      <main>
        <ContentPreview />
      <Footer/>
      </main>
    </Layout>
  );
}


