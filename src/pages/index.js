import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';


function ResourcesHeader() {
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          AutoGence Resources Center
        </Heading>
        <p className="hero__subtitle">
          Welcome to the AutoGence Resources Center — how can we help you get started?
        </p>
        <p className={styles.heroDescription}>
          Browse the guides and API references for all AutoGence products, find tutorials to kick off your first project with Harfy, and connect with the community for help and collaboration.
        </p>

        {/* Quick Links */}
        <div className={styles.quickLinks}>
          <Link className={styles.quickLinkCard} to="/docs/components/actuator-api">
            <div className={styles.quickLinkContent}>
              <h3>Actuator Control API</h3>
              <p>Complete API reference for controlling joint actuators</p>
            </div>
          </Link>
          <Link className={styles.quickLinkCard} to="/docs/quick-start/harfy-setup">
            <div className={styles.quickLinkContent}>
              <h3>Harfy Quick Start</h3>
              <p>Get your Harfy robot up and running</p>
            </div>
          </Link>
          <Link className={styles.quickLinkCard} to="/docs/api/intro">
            <div className={styles.quickLinkContent}>
              <h3>API Reference</h3>
              <p>Documentation for all AutoGence APIs</p>
            </div>
          </Link>
        </div>
      </div>
    </header>
  );
}

function DocumentationSections() {
  return (
    <section className={styles.documentationSection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>Documentation</Heading>
        <p className={styles.sectionIntro}>Comprehensive guides and references for all AutoGence products</p>

        <div className={styles.docsGrid}>
          {/* Harfy Documentation */}
          <div className={styles.docsCategory}>
            <div className={styles.docsHeader}>
              <h3 className={styles.docsTitle}>Harfy Documentation</h3>
              <p className={styles.docsDescription}>Complete guides and API references for using Harfy</p>
            </div>
            <ul className={styles.docsList}>
              <li><strong>Getting Started with Harfy</strong> - Step-by-step guide from unboxing to first program (Coming Soon)</li>
              <li><strong>Harfy User Manual</strong> - Hardware specs, safety, and maintenance (Coming Soon)</li>
              <li><strong>Programming Harfy</strong> - Control APIs and ROS2 interface with examples (Coming Soon)</li>
              <li><strong>Harfy SDK / Low-code API</strong> - High-level API for movements and behaviors (Coming Soon)</li>
            </ul>
          </div>

          {/* Robot Domain Controller */}
          <div className={styles.docsCategory}>
            <div className={styles.docsHeader}>
              <h3 className={styles.docsTitle}>Robot Domain Controller</h3>
              <p className={styles.docsDescription}>Hardware controller for real-time robot control</p>
            </div>
            <ul className={styles.docsList}>
              <li><strong>Domain Controller Setup</strong> - Hardware installation and configuration (Coming Soon)</li>
              <li><strong>ROS2 Integration</strong> - Connecting with ROS2 systems (Coming Soon)</li>
              <li><strong>EtherCAT Configuration</strong> - Real-time communication setup (Coming Soon)</li>
              <li><strong>Troubleshooting Guide</strong> - Common issues and solutions (Coming Soon)</li>
            </ul>
          </div>

          {/* Joint Actuators & Sensors */}
          <div className={styles.docsCategory}>
            <div className={styles.docsHeader}>
              <h3 className={styles.docsTitle}>Actuators & Sensors</h3>
              <p className={styles.docsDescription}>Smart motors and sensors for robotics projects</p>
            </div>
            <ul className={styles.docsList}>
              <li><Link to="/docs/components/actuator-api">Actuator Control API</Link> - Complete API reference for controlling joint actuators</li>
              <li><strong>Joint Actuator Specifications</strong> - Torque, payload, and interface details (Coming Soon)</li>
              <li><strong>Sensor Configuration</strong> - Camera, IMU, and depth sensor setup (Coming Soon)</li>
              <li><strong>Component Integration</strong> - Connecting actuators and sensors (Coming Soon)</li>
            </ul>
          </div>

          {/* Software Platform
          <div className={styles.docsCategory}>
            <div className={styles.docsHeader}>
              <h3 className={styles.docsTitle}>☁️ Software Platform</h3>
              <p className={styles.docsDescription}>OTA updates and robot management system</p>
            </div>
            <ul className={styles.docsList}>
              <li><strong>OTA Client Installation</strong> - Installing update client on robots (Coming Soon)</li>
              <li><strong>Management Dashboard</strong> - Fleet monitoring and control (Coming Soon)</li>
              <li><strong>PKI Setup</strong> - Security and authentication configuration (Coming Soon)</li>
              <li><strong>Teleoperation Setup</strong> - Remote control configuration (Coming Soon)</li>
            </ul>
          </div> */}
        </div>
      </div>
    </section>
  );
}

function QuickStartTutorials() {
  return (
    <section className={clsx(styles.tutorialsSection, styles.sectionAlt)}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>Quick Start Guides & Tutorials</Heading>
        <p className={styles.sectionIntro}>Hands-on learning with step-by-step instructions</p>

        <div className={styles.tutorialCategories}>
          {/* Quick Start Guides */}
          <div className={styles.tutorialCategory}>
            <h3 className={styles.categoryTitle}>Quick Start Guides</h3>
            <div className={styles.tutorialGrid}>
              <div className={styles.tutorialCard}>
                <div className={styles.tutorialHeader}>
                  <span className={clsx(styles.tutorialBadge, styles.beginner)}>Beginner</span>
                  <span className={styles.tutorialTime}>10 min</span>
                </div>
                <h4 className={styles.tutorialTitle}>Harfy in 10 Minutes</h4>
                <p className={styles.tutorialDescription}>Get your Harfy robot assembled, powered on, and running its first demo program.</p>
                <div className={styles.tutorialTech}>
                  <span className={styles.techTag}>Hardware</span>
                  <span className={styles.techTag}>Basic Setup</span>
                </div>
                <Link to="/docs/quick-start/harfy-setup" className="button button--primary">Start Guide</Link>
              </div>

              <div className={styles.tutorialCard}>
                <div className={styles.tutorialHeader}>
                  <span className={clsx(styles.tutorialBadge, styles.beginner)}>Beginner</span>
                  <span className={styles.tutorialTime}>5 min</span>
                </div>
                <h4 className={styles.tutorialTitle}>First Movement Command</h4>
                <p className={styles.tutorialDescription}>Send your first movement command to Harfy using the API or ROS2.</p>
                <div className={styles.tutorialTech}>
                  <span className={styles.techTag}>API</span>
                  <span className={styles.techTag}>ROS2</span>
                </div>
                <button className="button button--primary" disabled>Coming Soon</button>
              </div>

              <div className={styles.tutorialCard}>
                <div className={styles.tutorialHeader}>
                  <span className={clsx(styles.tutorialBadge, styles.intermediate)}>Intermediate</span>
                  <span className={styles.tutorialTime}>15 min</span>
                </div>
                <h4 className={styles.tutorialTitle}>OTA Update in Action</h4>
                <p className={styles.tutorialDescription}>Learn how to push software updates to your Harfy robot.</p>
                <div className={styles.tutorialTech}>
                  <span className={styles.techTag}>Software</span>
                  <span className={styles.techTag}>Updates</span>
                </div>
                <button className="button button--primary" disabled>Coming Soon</button>
              </div>
            </div>
          </div>

          {/* Advanced Tutorials */}
          <div className={styles.tutorialCategory}>
            <h3 className={styles.categoryTitle}>Project Tutorials</h3>
            <div className={styles.tutorialGrid}>
              <div className={styles.tutorialCard}>
                <div className={styles.tutorialHeader}>
                  <span className={clsx(styles.tutorialBadge, styles.intermediate)}>Intermediate</span>
                  <span className={styles.tutorialTime}>30 min</span>
                </div>
                <h4 className={styles.tutorialTitle}>Teleoperation Tutorial</h4>
                <p className={styles.tutorialDescription}>Set up remote control of Harfy using a gamepad or web interface.</p>
                <div className={styles.tutorialTech}>
                  <span className={styles.techTag}>WebRTC</span>
                  <span className={styles.techTag}>Control</span>
                </div>
                <button className="button button--primary" disabled>Coming Soon</button>
              </div>

              <div className={styles.tutorialCard}>
                <div className={styles.tutorialHeader}>
                  <span className={clsx(styles.tutorialBadge, styles.advanced)}>Advanced</span>
                  <span className={styles.tutorialTime}>45 min</span>
                </div>
                <h4 className={styles.tutorialTitle}>Vision-Language Navigation</h4>
                <p className={styles.tutorialDescription}>Use VLN models to give Harfy verbal navigation instructions.</p>
                <div className={styles.tutorialTech}>
                  <span className={styles.techTag}>AI/ML</span>
                  <span className={styles.techTag}>Navigation</span>
                </div>
                <button className="button button--primary" disabled>Coming Soon</button>
              </div>

              <div className={styles.tutorialCard}>
                <div className={styles.tutorialHeader}>
                  <span className={clsx(styles.tutorialBadge, styles.intermediate)}>Intermediate</span>
                  <span className={styles.tutorialTime}>25 min</span>
                </div>
                <h4 className={styles.tutorialTitle}>Customizing Behaviors</h4>
                <p className={styles.tutorialDescription}>Write new skills for Harfy like dances or custom tasks using the APIs.</p>
                <div className={styles.tutorialTech}>
                  <span className={styles.techTag}>Programming</span>
                  <span className={styles.techTag}>Behaviors</span>
                </div>
                <button className="button button--primary" disabled>Coming Soon</button>
              </div>

              <div className={styles.tutorialCard}>
                <div className={styles.tutorialHeader}>
                  <span className={clsx(styles.tutorialBadge, styles.advanced)}>Advanced</span>
                  <span className={styles.tutorialTime}>60 min</span>
                </div>
                <h4 className={styles.tutorialTitle}>Building a Custom Bot</h4>
                <p className={styles.tutorialDescription}>Use AutoGence components to build your own robot from scratch.</p>
                <div className={styles.tutorialTech}>
                  <span className={styles.techTag}>Hardware</span>
                  <span className={styles.techTag}>Custom Build</span>
                </div>
                <button className="button button--primary" disabled>Coming Soon</button>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function CommunitySupport() {
  return (
    <section className={styles.communitySection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>Community Support</Heading>
        <p className={styles.sectionIntro}>Get help, share projects, and connect with fellow developers</p>

        <div className={styles.communityGrid}>
          <div className={clsx(styles.communityCard, styles.discordCard)}>
            <div className={styles.communityHeader}>
              <div className={styles.communityIcon}></div>
              <h3>Join Our Discord</h3>
            </div>
            <p className={styles.communityDescription}>
              Have questions or need help troubleshooting? Join our Discord community to get support from AutoGence engineers and fellow developers.
            </p>
            <div className={styles.communityStats}>
              <span className={styles.stat}>Active Community</span>
              <span className={styles.stat}>Developers & Makers</span>
              <span className={styles.stat}>Technical Support</span>
            </div>
            <a href="https://discord.gg/autogence" className="button button--primary">Join Discord Community</a>
          </div>

          <div className={styles.communityCard}>
            <div className={styles.communityHeader}>
              <div className={styles.communityIcon}></div>
              <h3>FAQ</h3>
            </div>
            <p className={styles.communityDescription}>
              Find answers to frequently asked questions about Harfy, shipping, hardware warranty, and compatibility requirements.
            </p>
            <div className={styles.faqPreview}>
              <div className={styles.faqItem}>
                <strong>Q:</strong> What are Harfy's hardware requirements?
              </div>
              <div className={styles.faqItem}>
                <strong>Q:</strong> How do I update Harfy's firmware?
              </div>
              <div className={styles.faqItem}>
                <strong>Q:</strong> What programming languages are supported?
              </div>
            </div>
            <button className="button button--secondary" disabled>Coming Soon</button>
          </div>

          <div className={styles.communityCard}>
            <div className={styles.communityHeader}>
              <div className={styles.communityIcon}></div>
              <h3>Share Your Projects</h3>
            </div>
            <p className={styles.communityDescription}>
              Built something awesome with Harfy? Share your projects with the community and inspire other makers.
            </p>
            <div className={styles.projectShowcase}>
              <div className={styles.showcaseItem}>Harfy Dance Choreographer</div>
              <div className={styles.showcaseItem}>Home Assistant Integration</div>
              <div className={styles.showcaseItem}>VR Teleoperation Setup</div>
            </div>
            <button className="button button--secondary" disabled>Coming Soon</button>
          </div>
        </div>
      </div>
    </section>
  );
}

function OpenSourceProjects() {
  return (
    <section className={clsx(styles.openSourceSection, styles.sectionAlt)}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>Open Source Projects</Heading>
        <p className={styles.sectionIntro}>AutoGence is on GitHub! We openly share many components of our tech.</p>

        <div className={styles.githubOverview}>
          <div className={styles.githubStats}>
            <div className={styles.statCard}>
              <div className={styles.statNumber}>12+</div>
              <div className={styles.statLabel}>Repositories</div>
            </div>
            <div className={styles.statCard}>
              <div className={styles.statNumber}>500+</div>
              <div className={styles.statLabel}>Stars</div>
            </div>
            <div className={styles.statCard}>
              <div className={styles.statNumber}>50+</div>
              <div className={styles.statLabel}>Contributors</div>
            </div>
          </div>
          <p className={styles.githubIntro}>Check out and star our repositories below. We welcome contributions!</p>
        </div>

        <div className={styles.reposGrid}>
          <div className={clsx(styles.repoCard, styles.featured)}>
            <div className={styles.repoHeader}>
              <h3 className={styles.repoTitle}>
                <span className={styles.repoIcon}></span>
                Harfy Control SDK
              </h3>
              <div className={styles.repoMeta}>
                <span className={styles.language}>Python</span>
                <span className={styles.stars}>123</span>
              </div>
            </div>
            <p className={styles.repoDescription}>
              The high-level API/SDK for controlling Harfy robot movements, sensor access, and behaviors. Open and free for the community.
            </p>
            <div className={styles.repoTags}>
              <span className={styles.tag}>API</span>
              <span className={styles.tag}>SDK</span>
              <span className={styles.tag}>Python</span>
              <span className={styles.tag}>ROS2</span>
            </div>
            <div className={styles.repoActions}>
              <a href="https://github.com/autogence/harfy-sdk" className="button button--primary">View on GitHub</a>
              <button className="button button--secondary" disabled>Coming Soon</button>
            </div>
          </div>

          <div className={styles.repoCard}>
            <div className={styles.repoHeader}>
              <h3 className={styles.repoTitle}>
                <span className={styles.repoIcon}></span>
                Harfy Teleop
              </h3>
              <div className={styles.repoMeta}>
                <span className={styles.language}>TypeScript</span>
                <span className={styles.stars}>87</span>
              </div>
            </div>
            <p className={styles.repoDescription}>
              Remote control interface for Harfy using keyboard, gamepad, or web interface. WebRTC-based with low latency.
            </p>
            <div className={styles.repoTags}>
              <span className={styles.tag}>WebRTC</span>
              <span className={styles.tag}>Control</span>
              <span className={styles.tag}>React</span>
            </div>
            <div className={styles.repoActions}>
              <a href="https://github.com/autogence/harfy-teleop" className="button button--primary">View on GitHub</a>
              <button className="button button--secondary" disabled>Coming Soon</button>
            </div>
          </div>

          <div className={styles.repoCard}>
            <div className={styles.repoHeader}>
              <h3 className={styles.repoTitle}>
                <span className={styles.repoIcon}></span>
                VLN & VLA Models
              </h3>
              <div className={styles.repoMeta}>
                <span className={styles.language}>Python</span>
                <span className={styles.stars}>234</span>
              </div>
            </div>
            <p className={styles.repoDescription}>
              Pre-trained models for Vision-Language Navigation and Vision-Language Action. Enable natural language control of robots.
            </p>
            <div className={styles.repoTags}>
              <span className={styles.tag}>AI/ML</span>
              <span className={styles.tag}>Vision</span>
              <span className={styles.tag}>NLP</span>
              <span className={styles.tag}>PyTorch</span>
            </div>
            <div className={styles.repoActions}>
              <a href="https://github.com/autogence/vln-models" className="button button--primary">View on GitHub</a>
              <button className="button button--secondary" disabled>Coming Soon</button>
            </div>
          </div>

          <div className={styles.repoCard}>
            <div className={styles.repoHeader}>
              <h3 className={styles.repoTitle}>
                <span className={styles.repoIcon}></span>
                Hardware Interface
              </h3>
              <div className={styles.repoMeta}>
                <span className={styles.language}>C++</span>
                <span className={styles.stars}>56</span>
              </div>
            </div>
            <p className={styles.repoDescription}>
              Low-level code for domain controller and actuator firmware. Real-time control and EtherCAT communication.
            </p>
            <div className={styles.repoTags}>
              <span className={styles.tag}>Firmware</span>
              <span className={styles.tag}>EtherCAT</span>
              <span className={styles.tag}>Real-time</span>
            </div>
            <div className={styles.repoActions}>
              <a href="https://github.com/autogence/hardware-interface" className="button button--primary">View on GitHub</a>
              <button className="button button--secondary" disabled>Coming Soon</button>
            </div>
          </div>
        </div>

        <div className={styles.contributionSection}>
          <h3 className={styles.contributionTitle}>Want to Contribute?</h3>
          <p className={styles.contributionDescription}>
            We welcome pull requests! Whether you're developing a new feature for Harfy or fine-tuning a model, we welcome contributions. Beta testers and early adopters are invited to give feedback and help us improve.
          </p>
          <div className={styles.contributionActions}>
            <a href="https://github.com/autogence/contributing" className="button button--primary">Contribution Guidelines</a>
            <a href="https://github.com/autogence" className="button button--secondary">Good First Issues</a>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="AutoGence Resources Center"
      description="Browse guides, API references, and tutorials for AutoGence products. Get started with Harfy robot, explore our hardware components, and connect with the community.">
      <ResourcesHeader />
      <main>
        <DocumentationSections />
        <QuickStartTutorials />
        <CommunitySupport />
        <OpenSourceProjects />
      </main>
    </Layout>
  );
}
