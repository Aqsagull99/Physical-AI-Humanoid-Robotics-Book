import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

import styles from './Footer.module.css';

function FooterLink({
  to,
  href,
  label,
  prependBaseUrlToHref,
  ...props
}: {
  to?: string;
  href?: string;
  label?: string;
  prependBaseUrlToHref?: string;
} & React.AnchorHTMLAttributes<HTMLAnchorElement>): ReactNode {
  const toUrl = useBaseUrl(to);
  const normalizedHref = useBaseUrl(href, { absolute: true });

  return (
    <Link
      className={styles.footerLink}
      {...(href
        ? {
            href: prependBaseUrlToHref ? normalizedHref : href,
          }
        : {
            to: toUrl,
          })}
      {...props}>
      {label}
    </Link>
  );
}

export default function Footer(): ReactNode {
  const {siteConfig} = useDocusaurusContext();

  return (
    <footer className={clsx('footer', styles.footer)}>
      <div className="container container--fluid padding-horiz--md">
        <div className="row footer__links">
          <div className="col col--3">
            <div className={styles.footerSection}>
              <h4 className={styles.footerTitle}>{siteConfig.title}</h4>
              <p className={styles.footerDescription}>
                {siteConfig.tagline}
              </p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.footerSection}>
              <h4 className={styles.footerTitle}>Modules</h4>
              <ul className={styles.footerList}>
                <li><FooterLink to="/docs/ros2" label="ROS 2 & Robotic Nervous System" /></li>
                <li><FooterLink to="/docs/digital-twin" label="Digital Twin & Simulation" /></li>
                <li><FooterLink to="/docs/ai-robot-brain" label="AI-Robot Brain" /></li>
                <li><FooterLink to="/docs/vla-capstone" label="Vision-Language-Action & Capstone" /></li>
              </ul>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.footerSection}>
              <h4 className={styles.footerTitle}>Resources</h4>
              <ul className={styles.footerList}>
                <li><FooterLink to="/docs/references" label="References" /></li>
                <li><FooterLink to="/docs/hardware-lab-setup" label="Hardware & Lab Setup" /></li>
                <li><FooterLink to="/docs/weekly-learning-plan" label="Weekly Learning Plan" /></li>
              </ul>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.footerSection}>
              <h4 className={styles.footerTitle}>Connect</h4>
              <ul className={styles.footerList}>
                <li><a href="https://github.com/physical-ai-book/physical-ai-humanoid-robotics" target="_blank" rel="noopener noreferrer" className={styles.footerLink}>GitHub</a></li>
                <li><a href="mailto:info@physical-ai-book.com" className={styles.footerLink}>Contact</a></li>
              </ul>
            </div>
          </div>
        </div>
        <div className={styles.footerBottom}>
          <div className="text--center">
            <p className={styles.copyright}>
              {siteConfig.themeConfig.footer?.copyright || `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`}
            </p>
          </div>
        </div>
      </div>
    </footer>
  );
}


