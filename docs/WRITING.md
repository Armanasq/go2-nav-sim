# Writing guide

How text in this repository is written. It applies to the tutorial, the module READMEs, the website and commit messages.

## What a sentence must do

- State one thing a reader can act on or check. If it cannot be checked, cut it or say where the evidence is.
- Give numbers with their source: the file, the command, or the run they came from.
- Say what did not work. A failure with its cause is worth more than a success claim.
- Prefer the concrete noun and the plain verb: "the planner found no path", not "path planning challenges were encountered".

## Patterns to remove

Each of these is common in generated or padded text. One alone proves nothing; several together make a page read as filler.

| Pattern | Example to avoid | Instead |
|---|---|---|
| Inflated or promotional words | robust, seamless, powerful, cutting-edge, comprehensive, leverage, delve, intricate, meticulous, pivotal, crucial | say what it does and how well, with a number |
| Significance tacked on with "-ing" | "…, highlighting the importance of calibration" | end the sentence, or state the consequence as a fact |
| Contrast framing | "not just X, but Y"; "it's not about X, it's about Y" | state Y |
| Lists of three by habit | "fast, reliable and scalable" | as many items as there are |
| Editorial asides | "it is important to note that", "notably", "interestingly" | delete; if it matters, the sentence already shows it |
| Essay connectives | moreover, furthermore, additionally, in conclusion | start the sentence with its subject |
| Em-dash asides | "the estimator — which uses the gyro — …" | a comma, parentheses, or two sentences |
| Bold labels on every bullet | "**Speed:** …" repeated down a list | a table if the items share fields; plain bullets otherwise |
| Title Case Headings | "Setting Up The Workspace" | sentence case |
| Summaries of what was just said | "In summary, this chapter showed…" | stop when the content ends |
| Vague hedges and attributions | "it is widely believed", "may potentially" | cite, measure, or say you did not check |
| Rhetorical questions and "Let's" | "Why does this matter? Let's dive in." | the answer, directly |

## Design follows the same rule

The website is for reading and checking. No gradients, glow, glass effects, decorative icons or badges. Colour carries
meaning (ground truth grey, odometry blue, AMCL violet, plan amber; validated / partial / planned) and nothing else.

## Sources for this guide

- Wikipedia, "Signs of AI writing" (WikiProject AI Cleanup field guide), 2025.
- D. Kobak et al., "Delving into LLM-assisted writing in biomedical publications through excess vocabulary", arXiv:2406.07016.
- G. Orwell, "Politics and the English Language", 1946: the six rules still cover most of the table above.
