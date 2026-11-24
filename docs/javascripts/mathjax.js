// MathJax settings for pymdownx.arithmatex (generic mode), per the Material for MkDocs docs.
window.MathJax = {
  loader: {
    load: ["[tex]/boldsymbol"],
  },
  tex: {
    packages: { "[+]": ["boldsymbol"] },
    inlineMath: [["\\(", "\\)"]],
    displayMath: [["\\[", "\\]"]],
    processEscapes: true,
    processEnvironments: true,
  },
  options: {
    ignoreHtmlClass: ".*|",
    processHtmlClass: "arithmatex",
  },
};

// Material swaps pages without a reload, so typeset each page again when it appears.
document$.subscribe(() => {
  if (!window.MathJax || !MathJax.startup || !MathJax.startup.output) return;
  MathJax.startup.output.clearCache();
  MathJax.typesetClear();
  MathJax.texReset();
  MathJax.typesetPromise();
});
