# Lab MCP installation and operation

See the [post-clone change inventory](../../POST_CLONE_CHANGES.md) for current
collision and tracking changes and validation limits. The original setup record
below predates those tests. In particular, the lab arm has seven enabled
colliders inside USD instances; ordinary traversal missed them. The workstation
loader now adds CAD-part box proxies and restarts physics when required.

Installed on 2026-10-26 (America/Phoenix). Repository commits and asset locations
are listed in README.md. No physical robot connection has been established.

## Launch the lab scene

Close the simulator normally before relaunching; preserve any scene edits first.
From `/home/kaleb/projects/RISE/BrickSim` run:

```bash
./scripts/run_rise_lab.sh
```

The **UR5e Digital Twin** panel should be present. Select **Quick Start +
Workstation** to load the supplied UR5e, RG2 gripper, workspace camera, and Vention
bench. The launcher configures the bundled ROS 2 Jazzy libraries and localhost
ROS discovery. It does not launch a physical robot driver. The lab extension
includes driver tools, but those were not invoked during setup.

The lab Quick Start is a single-arm, position-mirrored twin. Its source disables
gravity on the robot chain; it is not the same as a physically validated,
force-controlled cooperative manipulation model. It also changes collision
cooking settings. Record those changes when comparing physics experiments.

## Local Codex desktop

Two globally configured stdio MCP gateways are installed:

| Name | Executable | Bridge endpoint |
| --- | --- | --- |
| `rise-kit` | `../isaac-sim-mcp/.venv/bin/kit-mcp` | `127.0.0.1:9009` |
| `rise-isaac` | `../isaac-sim-mcp/.venv/bin/isaac-mcp` | `localhost:8766` |

The shared bridge provides general Kit inspection and Python execution. The lab
extension owns its separate bridge and exposes namespaced tools such as
`ur5e__quick_start_with_workstation`. This is the current lab source's two-bridge
architecture, despite the generic repository's preferred single-bridge design.

Restart/reload the local Codex client or open a fresh local chat to load newly
registered MCP servers. The current chat's original tool catalog is not proof of
hot activation. Start Isaac Sim before discovery; if the gateway started first,
call its `refresh_tools` after the simulator starts. With multiple simulator
instances, verify the endpoint's PID; a second general bridge may use an
ephemeral port rather than 9009.

Checks:

```bash
codex mcp get rise-kit
codex mcp get rise-isaac
../isaac-sim-mcp/.venv/bin/python -m kit_mcp.client status --port 9009
../isaac-sim-mcp/.venv/bin/python -m kit_mcp.client status --port 8766
```

Both actual MCP stdio handshakes and tool discovery passed: 8 generic tools and
30 lab-endpoint tools, including builtins and gateway refresh. Workstation
loading and the full simulated Quick Start returned success after compatibility
fixes. The simulator startup test used its bundled ROS libraries. No real robot
driver tool was called, and motion tracking/contact performance remains untested.

## Regular ChatGPT chats: tunnel created; runtime pending

ChatGPT web does not read local Codex MCP configuration. Use its **Add custom MCP
server → Tunnel** route for a private local server. An ordinary TCP bridge port
such as 8766 is **not** a ChatGPT MCP URL. Do not expose either raw bridge publicly.

The official OpenAI tunnel client 0.0.18 is downloaded under
`../tools/tunnel-client-v0.0.18/`; its Linux archive SHA256 was checked against
the official release checksums. Binary quickstart/help were inspected.

After explicit user approval, **RISE Isaac Sim** was created in the Personal
organization and associated with the single available ChatGPT workspace.
Tunnel ID: `tunnel_6ae0430847888191a155fc8c3f92f1b3`.
The local stdio profile is configured at
`outputs/rise/tunnel-profile/rise-isaac.yaml`; it references the runtime secret
through `env:CONTROL_PLANE_API_KEY` and uses a loopback-only health listener.
No secret is stored in that profile.

The key form is prepared as **RISE Isaac Sim runtime**, current Default project,
Restricted permissions with only **Tunnels Read + Use**, and **30-day expiration**.
Automatic approval review blocked the final key-creation click, requiring
separate credential-creation confirmation. The user completed that click and
reported the key ready; the secret has not been read by the agent. Runtime
authentication, tunnel readiness, and the ChatGPT connection are not verified.

After the user creates the key, start the simulator in one terminal and the
tunnel in another:

```bash
cd /home/kaleb/projects/RISE/BrickSim
./scripts/run_rise_lab.sh
```

```bash
cd /home/kaleb/projects/RISE/BrickSim
./scripts/start_chatgpt_tunnel.sh
```

The second script checks the simulator endpoint, prompts for the key with input
echo disabled, runs the tunnel doctor, then runs the tunnel in the foreground.
The secret is supplied in process memory through the environment, not saved in
the repository or placed in a shell command/history. Keep this terminal open
while ChatGPT is using the connection. Ctrl+C stops the tunnel. It is not an
installed background service.

The remaining account-side steps require access to the intended Platform
organization and ChatGPT workspace:

1. Sign in to [Platform tunnel settings](https://platform.openai.com/settings/organization/tunnels).
   Create or select the intended tunnel and associate the ChatGPT workspace.
   Creation requires Tunnels Read + Manage; use requires Tunnels Read + Use.
2. Create/use a restricted runtime key with the required tunnel permissions.
   Enter it locally as `CONTROL_PLANE_API_KEY`; do not paste it into chat, save
   it in this repository, or use an administrative key as the runtime key.
3. With the actual tunnel ID, initialize a named stdio profile:

   ```bash
   ../tools/tunnel-client-v0.0.18/tunnel-client init \
     --profile rise-isaac \
     --profile-dir "$PWD/outputs/rise/tunnel-profile" \
     --tunnel-id YOUR_ACTUAL_TUNNEL_ID \
     --mcp-command "$PWD/scripts/rise_isaac_mcp.sh"
   ```

   Consult current `doctor --help` and `run --help` for profile-directory flags,
   then validate and start the profile with the runtime key supplied through its
   environment reference. Report connection success only when health/readiness
   checks pass; do not use a fake tunnel ID to imply installation completion.
4. While the simulator and tunnel client are running, open
   [ChatGPT Plugins](https://chatgpt.com/plugins), select Add custom MCP server,
   choose Tunnel, and select the same tunnel. Complete account permissions and
   connection review, then test a read-only status call before scene changes.

The signed-in Chrome page is user-controlled until it is shared through a
supported browser attachment. A screenshot is enough to guide the next click,
but does not give browser-control access or finish account authorization.

References: [OpenAI MCP configuration](https://learn.chatgpt.com/docs/extend/mcp?surface=cli),
[private tunnel setup](https://developers.openai.com/api/docs/guides/secure-mcp-tunnels),
[ChatGPT custom MCP connection](https://developers.openai.com/api/docs/guides/custom-mcp-server).

## Local compatibility patches

The lab repository remains on its inspected upstream commit plus explicit local
changes in `exts/ur5e-dt/ur5e_dt/extension.py` and `isaac_mcp/server.py`:

- Register tools with the newer owner-scoped bridge API under `ur5e`.
- Normalize that prefix before existing checkpoint gates and output-dir policy.
- Initialize a Python physics context even if a USD PhysicsScene already exists.
- Delegate `isatty()` through the log redirector so uvicorn logging can start.

Preserve/review these patches before pulling updates. They have not been pushed
or presented as upstream fixes. Gate-policy behavior must remain intact; do not
remove the namespace normalization when updating registration.

## Settings and rollback

The installer added both extension directories and autoload entries to
`~/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/user.config.json`, backing up the
previous file as `user.config.json.bak.1793080747`. Apply persistent extension
registration only while Isaac Sim is closed, since graceful shutdown rewrites
these settings. Remove Codex entries with `codex mcp remove rise-kit` and
`codex mcp remove rise-isaac` if requested. The generic installer's `--remove`
can undo its registration using the same folder/extension arguments; it should
not run automatically as part of cleanup.
