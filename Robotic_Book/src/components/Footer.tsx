import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

import styles from './Footer.module.css';

type FooterLinkProps = {
  to?: string;
  href?: string;
  label: string;
  prependBaseUrlToHref?: boolean;
} & React.AnchorHTMLAttributes<HTMLAnchorElement>;

function FooterLink({
  to,
  href,
  label,
  prependBaseUrlToHref,
  ...props
}: FooterLinkProps): ReactNode {
  const toUrl = useBaseUrl(to || '/');
  const normalizedHref = href
    ? useBaseUrl(href, { absolute: true })
    : undefined;

  return (
    <Link
      className={styles.footerLink}
      {...(href
        ? {
            href: prependBaseUrlToHref ? normalizedHref : href,
          }
        : { to: toUrl })}
      {...props}
    >
      {label}
    </Link>
  );
}

export default function Footer(): ReactNode {
  const { siteConfig } = useDocusaurusContext();

  return (
    <footer className={clsx(styles.footer)}>
      <div className="">
        <div className="row">
          {/* Brand */}
          <div className="col col--4">
            <div className={styles.footerSection}>
              <h4 className={styles.footerTitle}>{siteConfig.title}</h4> 
               <p className={styles.footerDescription}>
                {siteConfig.tagline}
              </p> 
            </div>
          </div>

          {/* Navigation */}
          <div className="col col--2">
            <div className={styles.footerSection}>
              <h4 className={styles.footerTitle}>Navigation</h4>
              <ul className={styles.footerList}>
                <li><FooterLink to="/" label="Home" /></li>
                <li><FooterLink to="/docs/intro" label="Textbook" /></li>
                <li><FooterLink to="/about" label="About" /></li>
              </ul>
            </div>
          </div>

          {/* Modules */}
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

          {/* Connect */}
          <div className="col col--3">
            <div className={styles.footerSection}>
              <h4 className={styles.footerTitle}>Connect</h4>
              <ul className={styles.footerList}>
                <li>
                  <a
                    className={styles.footerLink}
                    href="https://github.com/physical-ai-book/physical-ai-humanoid-robotics"
                    target="_blank"
                    rel="noopener noreferrer"
                  >
                    GitHub
                  </a>
                </li>
                <li>
                  <a
                    className={styles.footerLink}
                    href="mailto:info@physical-ai-book.com"
                  >
                    Contact
                  </a>
                </li>
              </ul>
            </div>
          </div>
        </div>

        {/* Bottom */}
        <div className={styles.footerBottom}>
          <p className={styles.copyright}>
            {`© ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`}
          </p>
        </div>
      </div>
    </footer>
  );
}
