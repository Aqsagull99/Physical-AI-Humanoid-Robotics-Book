<!--
Sync Impact Report:
- Version change: 1.0.0 → 2.0.0
- List of modified principles:
  - Accuracy → Accuracy (updated)
  - Clarity → Clarity (updated)
  - Reproducibility → Reproducibility (updated)
  - Rigor → Rigor (updated)
  - Added: Grounding & Non-hallucination
  - Added: Privacy & Minimal Data Retention
- Added sections: Projects, RAG Chatbot Standards, Constraints, Success Criteria, Deliverables, Testing & Validation, Operational Requirements, Additional Instructions for AI/Authoring Agents
- Removed sections: None
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->
# Constitution for AI/Spec-Driven Book Creation + Integrated RAG Chatbot Projects

## Projects

### 1. AI/Spec-Driven Book Creation
- Write a book using Docusaurus and deploy it to GitHub Pages.
- Authoring workflow uses Spec-Kit Plus (https://github.com/panaversity/spec-kit-plus/) and Claude Code (https://www.claude.com/product/claude-code).

### 2. Integrated RAG Chatbot Development
- Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book site.
- Chatbot must answer questions about the book's content.
- Chatbot must support answering based *only* on text the user selects/highlights.

---

## Core Principles
- **Accuracy:** All information must be verified using primary or highly reliable sources.
- **Clarity:** Writing and chatbot responses must be clear and suitable for a computer science audience.
- **Reproducibility:** Every claim must be traceable to its source; include links/citations.
- **Rigor:** Prefer peer-reviewed articles, official documentation, or authoritative publications.
- **Grounding & Non-hallucination:** Chatbot responses must be strictly grounded in retrieved materials; hallucinations are unacceptable.
- **Privacy & Minimal Data Retention:** User data is never stored beyond session needs and explicit consent.

---

## Key Standards

### Book (Project 1)
- All factual claims must be traceable to sources.
- Citation format: APA style.
- Source types: Minimum 50% peer-reviewed articles.
- Plagiarism: 0% tolerance; all content must be original or properly cited.
- Writing clarity: Flesch–Kincaid grade 10–12 readability.
- Language: Professional, academic tone suitable for technical readers.
- Word count: 5,000–7,000 words.
- Minimum 15 sources.
- Output formats:
  - Docusaurus-compatible Markdown (primary)
  - PDF export with embedded citations
- Deployment:
  - Docusaurus site deployable to GitHub Pages
  - Repository structure and CI/CD notes included

---

### RAG Chatbot (Project 2)

#### Functional Requirements
- Must follow strict Retrieval-Augmented Generation (RAG) rules.
- Answers must be derived *only* from retrieved book content.
- Must support **selection-only mode**, where answers are based solely on user-highlighted text.

#### Approved Tech Stack
- **Backend:** FastAPI
- **LLM / Embeddings Provider:** OpenAI *or* Cohere
- **Vector Store:** Qdrant Cloud (Free Tier)
- **Metadata Store:**
  - Neon Serverless Postgres (preferred), or
  - Qdrant payload metadata when Neon is omitted
- **Frontend:** Embedded chatbot UI in Docusaurus (React)

#### Embedding & Retrieval
- Produce embeddings for all book content.
- Supported models (example):
  - Cohere Embed models *or*
  - OpenAI Embedding models
- Retrieval must support:
  - Whole-book semantic search
  - User-selected text–restricted retrieval
- When selection mode is active, no external or unrelated chunks may be used.

#### Response Grounding
- Every chatbot answer must include:
  - Source ID(s)
  - Highlighted excerpt(s) or page/paragraph pointer
  - Optional APA citation when relevant
- If an answer cannot be found, the chatbot must respond with:
  > "I couldn't find a passage that directly answers this. Here are the closest related sources…"

#### Performance & Reliability
- Reasonable latency suitable for interactive web usage.
- Top-K retrieval and caching strategy documented.
- Embedding model name and version stored in metadata for reproducibility.

#### Security & Privacy
- API keys and secrets must not be committed to the repository.
- All communication must use TLS.
- No persistent storage of user queries or PII without explicit consent.
- A privacy notice must be displayed in the UI.

---

## Constraints
- Content consistency between the book and chatbot is mandatory.
- Only approved book sources may be used for embeddings.
- Chatbot must not hallucinate or infer beyond retrieved text.
- Code must be production-ready, typed, validated, and documented.
- Hosting:
  - Frontend: GitHub Pages
  - Backend & databases: cloud-hosted with deployment guide provided

---

## Success Criteria

### Book
- All claims verified and cited using APA style.
- Zero plagiarism.
- Docusaurus site fully deployable and publicly accessible.
- Professionally designed UI (homepage, header/footer, CTA).
- Ready-for-publication quality.

### Chatbot
- ≥ 95% of sampled responses correctly grounded in retrieved passages.
- Selection-only queries strictly limited to highlighted text.
- Fully embedded chatbot UI (desktop + mobile).
- End-to-end pipeline validated:
  - Ingestion → embedding → Qdrant → retrieval → LLM → cited response
- Logs demonstrate retrieval provenance for auditability.

---

## Deliverables
- Live Docusaurus site (GitHub Pages URL).
- PDF export with APA citations.
- FastAPI RAG service (ingestion + query endpoints).
- Qdrant collection with embeddings and ingestion scripts.
- Neon Postgres schema (if used) and deployment documentation.
- Chatbot UI embedded in Docusaurus with selection-to-chat flow.
- Architecture diagram, deployment guide, and validation reports.

---

## Testing & Validation
- Book:
  - Citation completeness
  - Plagiarism scan
  - Readability analysis
- Chatbot:
  - Grounding accuracy tests
  - Precision@K / Recall@K retrieval metrics
  - Selection-only mode tests
  - Load and latency testing
  - Security and privacy audit

---

## Operational Requirements
- Ingestion pipeline:
  - Canonical Markdown → chunking → embeddings → Qdrant
  - Metadata stored in Neon or vector payloads
- FastAPI endpoints:
  - POST /query
  - POST /admin/ingest
  - GET /health
- Prompt templates and agent rules must enforce citation and provenance.
- CI/CD pipelines documented for frontend and backend.

---

## Additional Instructions for AI / Authoring Agents
- Prioritize technical correctness over stylistic flair.
- Do not infer beyond provided sources.
- Always cite retrieved material in chatbot responses.
- Follow the Spec-Kit workflow strictly: Constitution → Spec → Plan → Task → Implement.

---

## Governance

This constitution is the single source of truth for project standards and principles. All work must adhere to it. Amendments require team consensus and must be documented here.

**Version**: 2.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-06