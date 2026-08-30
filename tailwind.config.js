/** Tailwind config used to compile assets/tailwind.css.
 *  Rebuild after editing classes in index.html:
 *    npm install
 *    npm run build:css
 */
module.exports = {
  content: ["./index.html"],
  theme: {
    extend: {
      colors: {
        void:      '#05070D',   // page background — near-black with a cold blue undertone
        panel:     '#0A0E19',   // recessed / solid panel background
        ink:       '#F4F6FB',   // primary text — near-white
        inksoft:   '#8B93A8',   // secondary text — muted slate
        line:      '#1B2233',   // hairline borders
        royal:     '#3654D6',   // primary accent — vivid royal blue
        royalDeep: '#16204F',   // deep royal blue — gradient depth / shadows
        silver:    '#B9C0D4',   // metallic silver-gray accent — premium detail
      },
      fontFamily: {
        display: ['"Space Grotesk"', 'sans-serif'],
        body: ['Inter', 'sans-serif'],
        mono: ['"JetBrains Mono"', 'monospace'],
      },
    }
  },
  plugins: [],
}
