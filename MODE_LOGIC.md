# Water Tank Controller: Mode Logic

This summary has been replaced by the client approval documents in [`docs/`](docs/):

| Document | Contents |
|---|---|
| `WT1.1_Functional_Specification_v<version>.docx` | Every rule with an ID, per-rule client OK checkbox and comment column, open points, revision history, approval block |
| `WT1.1_Test_Matrix_v<version>.xlsx` | Test cases per rule with status dropdown, summary per module, rules and decisions sheets |

Both are generated from `docs/spec_src/rules.json` and `docs/spec_src/tests.json`.
To change a rule: edit those files, raise `"version"` and add a `"history"` entry in `rules.json`, then run:

```
py -3 docs/spec_src/build_docs.py
```
