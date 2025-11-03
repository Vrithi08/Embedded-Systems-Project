# Contributing

Thanks for your interest in contributing. A couple of simple rules:

- Keep changes small and focused. Open a pull request with a short description of the intent.
- Follow the code style used in `pill_disp.c`. Add comments for any non-obvious logic.
- Run the basic syntax check locally before opening a PR:

  gcc -fsyntax-only -Wall pill_disp.c

- If the PR touches hardware-specific code, include the target board and toolchain used for testing.

If you need help, open an issue describing what you'd like to change.
