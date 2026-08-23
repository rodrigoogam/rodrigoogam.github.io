# Rodrigo Gaytan Miranda | Portfolio

Personal portfolio website for **Rodrigo Gaytan Miranda**, Mechatronics Engineer focused on Hardware-in-the-Loop (HIL) validation, embedded systems, industrial automation, and project management.

The site presents Rodrigo's experience, technical projects, skills, certifications, career direction, and contact information in a bilingual Spanish/English interface.

## Highlights

- Responsive single-page portfolio for desktop and mobile
- Spanish/English language toggle
- Experience timeline covering Satven, BME Budapest, ABB México, and Tecnológico de Monterrey
- Technical project showcase featuring CINVESTAV, SAE Aerodesign, and PLC automation work
- Skills and certifications section
- Interactive project gallery carousel
- Scroll progress indicator, chapter navigation, animated reveals, and reduced-motion support
- Downloadable English CV in PDF format
- Direct contact links for email, LinkedIn, and GitHub

## Built With

- HTML5
- Tailwind CSS via CDN
- Vanilla JavaScript
- Google Fonts: Space Grotesk, Inter, and JetBrains Mono

No build step or package installation is required.

## Project Structure

```text
.
├── index.html
├── Gemini_Generated_Image_md8zvpmd8zvpmd8z.png
├── assets/
│   ├── CV_Rodrigo_Gaytan_english .pdf
├── LICENSE
└── README.md
```

## Run Locally

Because this is a static site, it can be opened directly in a browser. A local server is recommended so that all browser features behave consistently:

```bash
python -m http.server 8000
```

Then open <http://localhost:8000>.

Alternatively, use the **Live Server** extension in VS Code and open `index.html`.

## Customization

Most content is contained in `index.html`:

1. Update the Spanish and English translations in the `translations` object near the bottom of the file.
2. Replace the contact links and CV path if personal details or assets change.
3. Add real project photos under `assets/gallery/` and update the gallery image paths in the multimedia section.
4. Update the page metadata in the `<head>` when the portfolio description changes.

The gallery currently includes fallback states for images that have not been added yet.

## Deployment

This repository is ready for GitHub Pages. To deploy it:

1. Push the repository to GitHub.
2. Open **Settings > Pages** in the repository.
3. Select **Deploy from a branch**.
4. Choose the main branch and the `/ (root)` folder.

GitHub Pages will serve `index.html` as the site entry point.

## Contact

- Email: [rgaytanmiranda9@gmail.com](mailto:rgaytanmiranda9@gmail.com)
- LinkedIn: [linkedin.com/in/rodrigoogam](https://www.linkedin.com/in/rodrigoogam/)
- GitHub: [github.com/rodrigoogam](https://github.com/rodrigoogam)