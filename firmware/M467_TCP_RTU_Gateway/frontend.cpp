// ================================================================
//  frontend.cpp  –  embedded web configuration UI (port httpPort)
//
//  Plain server-rendered HTML forms (not an AJAX/JSON SPA) — simplest,
//  most robust option for a C-string-built embedded web server serving
//  a small, fixed number of settings pages. Basic Auth reuses the same
//  base64Encode()/loginAuthBase64 pattern as the reference project.
// ================================================================
#include "frontend.h"

static EthernetServer *webServer = nullptr;

void frontendInit() {
  webServer = new EthernetServer(httpPort);
  webServer->begin();
  addLog("[WEB] server started on port " + String(httpPort));
}

// ================================================================
//  Small helpers
// ================================================================
static String urlDecode(const String &s) {
  String out; out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '+') {
      out += ' ';
    } else if (c == '%' && i + 2 < s.length()) {
      char hex[3] = { s[i + 1], s[i + 2], 0 };
      out += (char)strtol(hex, nullptr, 16);
      i += 2;
    } else {
      out += c;
    }
  }
  return out;
}

static bool formLocate(const String &body, const String &key, String &valueOut) {
  String search = key + "=";
  int idx = 0;
  while (idx < (int)body.length()) {
    int amp = body.indexOf('&', idx);
    String pair = (amp < 0) ? body.substring(idx) : body.substring(idx, amp);
    if (pair.startsWith(search)) { valueOut = urlDecode(pair.substring(search.length())); return true; }
    if (amp < 0) break;
    idx = amp + 1;
  }
  return false;
}
static String formField(const String &body, const String &key) {
  String v;
  formLocate(body, key, v);
  return v;
}
static bool formHasField(const String &body, const String &key) {
  String v;
  return formLocate(body, key, v);
}

static void printOption(EthernetClient &c, int value, const String &label, int current) {
  c.print("<option value='"); c.print(value); c.print("'");
  if (value == current) c.print(" selected");
  c.print(">"); c.print(label); c.println("</option>");
}

// ================================================================
//  Modicon 5-digit address <-> raw 0-based protocol address, for the
//  RTU Addr field. Wire-protocol code (modbus_rtu.cpp) always works in
//  raw 0-based addresses; only this web UI boundary speaks Modicon, since
//  that's the convention most Modbus tools (ModSim, etc.) display.
//    01 Coil     : 00001-09999  -> raw = modicon - 1
//    02 Discrete : 10001-19999  -> raw = modicon - 10001
//    03 Holding  : 40001-49999  -> raw = modicon - 40001
//    04 Input    : 30001-39999  -> raw = modicon - 30001
// ================================================================
static uint32_t rtuAddrToModicon(uint8_t func, uint16_t rawAddr) {
  switch (func) {
    case 1:  return (uint32_t)rawAddr + 1UL;
    case 2:  return (uint32_t)rawAddr + 10001UL;
    case 4:  return (uint32_t)rawAddr + 30001UL;
    default: return (uint32_t)rawAddr + 40001UL;   // 3 = Holding
  }
}
// The Modicon number alone unambiguously implies the function (that's the
// whole point of the convention), so we derive both func and raw address
// from it directly instead of trusting a separately-editable dropdown that
// could disagree with what was typed. Same ranges (5- and 6-digit) as the
// reference project's addrToFuncReg().
static void modiconToFuncAddr(uint32_t modicon, uint8_t &func, uint16_t &rawAddr) {
  if      (modicon >= 400001UL && modicon <= 499999UL) { func = 3; rawAddr = (uint16_t)(modicon - 400001UL); }
  else if (modicon >= 300001UL && modicon <= 399999UL) { func = 4; rawAddr = (uint16_t)(modicon - 300001UL); }
  else if (modicon >= 100001UL && modicon <= 109999UL) { func = 2; rawAddr = (uint16_t)(modicon - 100001UL); }
  else if (modicon >= 40001UL  && modicon <= 49999UL)  { func = 3; rawAddr = (uint16_t)(modicon - 40001UL); }
  else if (modicon >= 30001UL  && modicon <= 39999UL)  { func = 4; rawAddr = (uint16_t)(modicon - 30001UL); }
  else if (modicon >= 10001UL  && modicon <= 19999UL)  { func = 2; rawAddr = (uint16_t)(modicon - 10001UL); }
  else if (modicon >= 1UL      && modicon <= 9999UL)   { func = 1; rawAddr = (uint16_t)(modicon - 1UL); }
  else { func = 3; rawAddr = (uint16_t)modicon; }   // out-of-range fallback: treat as Holding, raw as-is
}

// ================================================================
//  HTTP response helpers
// ================================================================
static void sendHeader(EthernetClient &c) {
  c.println(F("HTTP/1.1 200 OK"));
  c.println(F("Connection: close"));
  c.println(F("Content-Type: text/html; charset=utf-8"));
  c.println();
}
static void sendRedirect(EthernetClient &c, const char *location) {
  c.println(F("HTTP/1.1 303 See Other"));
  c.print(F("Location: ")); c.println(location);
  c.println(F("Connection: close"));
  c.println();
}
static void send401(EthernetClient &c) {
  c.println(F("HTTP/1.1 401 Unauthorized"));
  c.println(F("WWW-Authenticate: Basic realm=\"Gateway\""));
  c.println(F("Connection: close"));
  c.println(F("Content-Type: text/html; charset=utf-8"));
  c.println();
  c.println(F("<h1>Please refresh to login</h1>"));
}
// Basic Auth has no real server-side session to invalidate — the browser
// just caches credentials per realm. The standard workaround: answer with
// a *different* realm string, which the browser treats as a separate
// protection space and won't auto-fill from its cache, forcing a fresh
// prompt (or the user can just close the tab).
static void send401Logout(EthernetClient &c) {
  c.println(F("HTTP/1.1 401 Unauthorized"));
  c.println(F("WWW-Authenticate: Basic realm=\"Gateway-logout\""));
  c.println(F("Connection: close"));
  c.println(F("Content-Type: text/html; charset=utf-8"));
  c.println();
  c.println(F("<h1>Logged out — close this tab, or enter any credentials to log back in.</h1>"));
}

static void navLink(EthernetClient &c, const char *href, const char *label, const char *activePath) {
  c.print(F("<a href='")); c.print(href); c.print(F("'"));
  if (strcmp(href, activePath) == 0) c.print(F(" class='active'"));
  c.print(F(">")); c.print(label); c.print(F("</a>"));
}

static void sendPageOpen(EthernetClient &c, const char *title, const char *activePath) {
  c.print(F("<!doctype html><html><head><meta charset='utf-8'><title>"));
  c.print(title);
  c.println(F("</title>"));
  // Minimal dark theme, inline CSS only — no external fonts/icons/JS, same
  // low-footprint plain-HTML approach as before, just restyled.
  c.println(F("<style>"
    "body{font-family:system-ui,-apple-system,Segoe UI,sans-serif;margin:0;padding:16px;background:#14161a;color:#d8dbe0}"
    "h2{margin:0 0 10px;font-size:19px;font-weight:600;color:#fff}"
    "nav{margin-bottom:14px;padding-bottom:10px;border-bottom:1px solid #262a33}"
    "nav a{color:#93a2b5;text-decoration:none;margin-right:16px;font-size:14px}"
    "nav a:hover{color:#4da3ff}"
    "nav a.active{color:#fff;font-weight:600;border-bottom:2px solid #4da3ff;padding-bottom:10px}"
    "nav a.logout{float:right;margin-right:0;color:#e08a8a}"
    "nav a.logout:hover{color:#ff6b6b}"
    "table{border-collapse:collapse;width:100%;font-size:13px}"
    "th,td{border:1px solid #262a33;padding:6px 8px;text-align:left}"
    "th{background:#1b1e24;color:#93a2b5;font-weight:600}"
    "tr:nth-child(even) td{background:#181b21}"
    "input,select{background:#1b1e24;color:#d8dbe0;border:1px solid #333844;border-radius:4px;padding:4px 6px;width:90px}"
    "input[type=checkbox]{width:auto}"
    "input[type=text],input[type=password]{width:200px}"
    "input:focus,select:focus{outline:none;border-color:#4da3ff}"
    "button{background:#4da3ff;color:#0b0d10;border:none;border-radius:4px;padding:7px 16px;"
    "font-weight:600;cursor:pointer;margin-top:10px}"
    "button:hover{background:#6bb4ff}"
    "a{color:#4da3ff}"
    "p{color:#93a2b5;font-size:13px}"
    "</style></head><body>"));
  c.print(F("<h2>")); c.print(sysName); c.println(F("</h2>"));
  c.print(F("<nav>"));
  navLink(c, "/",        "Dashboard", activePath);
  navLink(c, "/points",  "Points",    activePath);
  navLink(c, "/network", "Network",   activePath);
  navLink(c, "/serial",  "Serial",    activePath);
  navLink(c, "/system",  "System",    activePath);
  navLink(c, "/log",     "Log",       activePath);
  c.println(F("<a href='/logout' class='logout'>Logout</a></nav>"));
}
static void sendPageClose(EthernetClient &c) {
  c.println(F("</body></html>"));
}

// ================================================================
//  Dashboard
// ================================================================
static void sendDashboard(EthernetClient &c) {
  sendHeader(c);
  sendPageOpen(c, "Dashboard", "/");
  c.print(F("<p>IP: ")); c.print(Ethernet.localIP()); c.println(F("</p>"));
  c.print(F("<p>Uptime: ")); c.print(millis() / 1000); c.println(F(" s</p>"));
  c.print(F("<p>Modbus TCP clients active (last 15s): ")); c.print(mbTcpActiveClientCount()); c.println(F("</p>"));
  c.println(F("<table><tr><th>#</th><th>Name</th><th>Value</th><th>Raw</th><th>Status</th><th>Fail</th><th>Age(s)</th></tr>"));
  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    if (!points[i].enable) continue;
    PointRuntime &rt = pointState[i];
    c.print(F("<tr><td>")); c.print(i + 1); c.print(F("</td><td>")); c.print(points[i].name);
    c.print(F("</td><td>")); c.print(rt.value, 3); c.print(F("</td><td>"));
    c.print(rt.rawRegs[0]);
    if (formatRegCount(points[i].format) == 2) { c.print(','); c.print(rt.rawRegs[1]); }
    c.print(F("</td><td>")); c.print(rt.valid ? F("OK") : F("-")); c.print(F("</td><td>"));
    c.print(rt.failCount); c.print(F("</td><td>"));
    c.print(rt.valid ? (unsigned long)((millis() - rt.lastPollMs) / 1000) : 0);
    c.println(F("</td></tr>"));
  }
  c.println(F("</table>"));
  sendPageClose(c);
}

// ================================================================
//  Points
// ================================================================
static void sendPointsPage(EthernetClient &c) {
  sendHeader(c);
  sendPageOpen(c, "Points", "/points");
  c.println(F("<form method='POST' action='/points'><div style='overflow-x:auto'><table>"
              "<tr><th>#</th><th>En</th><th>Name</th><th>SlaveID</th><th>RTU Addr (Modicon)</th>"
              "<th>Format</th><th>Byte Order</th><th>Scale</th><th>Rate(ms)</th><th>Wr</th>"
              "<th>Unit ID</th><th>TCP Type</th><th>TCP Addr</th></tr>"));
  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    PointConfig &p = points[i];
    String pre = "p" + String(i) + "_";
    c.print(F("<tr><td>")); c.print(i + 1); c.print(F("</td>"));

    c.print(F("<td><input type=checkbox name='")); c.print(pre); c.print(F("en'"));
    if (p.enable) c.print(F(" checked")); c.print(F("></td>"));

    c.print(F("<td><input type=text maxlength=23 name='")); c.print(pre); c.print(F("name' value='"));
    c.print(p.name); c.print(F("'></td>"));

    c.print(F("<td><input type=number min=1 max=247 name='")); c.print(pre); c.print(F("sid' value='"));
    c.print(p.rtuSlaveId); c.print(F("'></td>"));

    c.print(F("<td><input type=number min=1 max=499999 name='")); c.print(pre); c.print(F("addr' value='"));
    c.print(rtuAddrToModicon(p.rtuFunc, p.rtuAddr)); c.print(F("'></td>"));

    // Read-only — derived from the Modicon address above, not independently editable.
    c.print(F("<td><select name='")); c.print(pre); c.print(F("fmt'>"));
    printOption(c, FMT_INT16,   "INT16",   p.format);
    printOption(c, FMT_UINT16,  "UINT16",  p.format);
    printOption(c, FMT_INT32,   "INT32",   p.format);
    printOption(c, FMT_UINT32,  "UINT32",  p.format);
    printOption(c, FMT_FLOAT32, "FLOAT32", p.format);
    printOption(c, FMT_BOOL,    "BOOL",    p.format);
    c.println(F("</select></td>"));

    c.print(F("<td><select name='")); c.print(pre); c.print(F("bo'>"));
    printOption(c, BO_BIG,         "Big",        p.byteOrder);
    printOption(c, BO_LITTLE,      "Little",     p.byteOrder);
    printOption(c, BO_BIG_SWAP,    "BigSwap",    p.byteOrder);
    printOption(c, BO_LITTLE_SWAP, "LittleSwap", p.byteOrder);
    c.println(F("</select></td>"));

    c.print(F("<td><input type=number step='any' name='")); c.print(pre); c.print(F("scale' value='"));
    c.print(p.scale, 6); c.print(F("'></td>"));

    c.print(F("<td><input type=number min=50 max=60000 name='")); c.print(pre); c.print(F("rate' value='"));
    c.print(p.pollIntervalMs); c.print(F("'></td>"));

    c.print(F("<td><input type=checkbox name='")); c.print(pre); c.print(F("wr'"));
    if (p.writable) c.print(F(" checked")); c.print(F("></td>"));

    c.print(F("<td><input type=number min=1 max=247 name='")); c.print(pre); c.print(F("uid' value='"));
    c.print(p.tcpUnitId); c.print(F("'></td>"));

    c.print(F("<td><select name='")); c.print(pre); c.print(F("rt'>"));
    printOption(c, TCP_COIL,     "Coil",     p.tcpRegType);
    printOption(c, TCP_DISCRETE, "Discrete", p.tcpRegType);
    printOption(c, TCP_HOLDING,  "Holding",  p.tcpRegType);
    printOption(c, TCP_INPUT,    "Input",    p.tcpRegType);
    c.println(F("</select></td>"));

    c.print(F("<td><input type=number min=0 max=65535 name='")); c.print(pre); c.print(F("taddr' value='"));
    c.print(p.tcpAddr); c.print(F("'></td>"));

    c.println(F("</tr>"));
  }
  c.println(F("</table></div><button type=submit>Save</button></form>"));
  c.println(F("<p>RTU Addr uses the standard 5-digit Modicon convention — the number itself picks the "
              "function: 00001-09999 Coil, 10001-19999 Discrete, 40001-49999 Holding, 30001-39999 Input "
              "(e.g. ModSim's \"40001\" label) — the function is derived from it automatically. TCP Addr "
              "(right-most column) is plain 0-based and is what a Modbus TCP client reads/writes — raw "
              "pass-through of the RTU register(s). Format/Byte Order/Scale only affect the dashboard "
              "value and RTU-side interpretation.</p>"));
  sendPageClose(c);
}

static void handlePointsPost(const String &body) {
  for (uint8_t i = 0; i < MAX_POINTS; i++) {
    String pre = "p" + String(i) + "_";
    PointConfig &p = points[i];
    p.enable = formHasField(body, pre + "en");
    String nm = formField(body, pre + "name");
    if (nm.length() > 0) {
      strncpy(p.name, nm.c_str(), sizeof(p.name) - 1);
      p.name[sizeof(p.name) - 1] = '\0';
    }
    String v;
    v = formField(body, pre + "sid");   if (v.length()) p.rtuSlaveId     = (uint8_t)v.toInt();
    // addr is entered Modicon-style in the UI (matches ModSim etc.); the function
    // code is derived from it, not independently selected — see modiconToFuncAddr().
    v = formField(body, pre + "addr");  if (v.length()) modiconToFuncAddr((uint32_t)v.toInt(), p.rtuFunc, p.rtuAddr);
    v = formField(body, pre + "fmt");   if (v.length()) p.format         = (uint8_t)v.toInt();
    v = formField(body, pre + "bo");    if (v.length()) p.byteOrder      = (uint8_t)v.toInt();
    v = formField(body, pre + "scale"); if (v.length()) p.scale          = v.toFloat();
    v = formField(body, pre + "rate");  if (v.length()) p.pollIntervalMs = (uint16_t)v.toInt();
    p.writable = formHasField(body, pre + "wr");
    v = formField(body, pre + "uid");   if (v.length()) p.tcpUnitId      = (uint8_t)v.toInt();
    v = formField(body, pre + "rt");    if (v.length()) p.tcpRegType     = (uint8_t)v.toInt();
    v = formField(body, pre + "taddr"); if (v.length()) p.tcpAddr        = (uint16_t)v.toInt();
    if (p.pollIntervalMs < 50) p.pollIntervalMs = 50;
  }
  savePointsConfig();
  pollEngineInit();
  addLog("[WEB] points updated");
}

// ================================================================
//  Network
// ================================================================
static void sendNetworkPage(EthernetClient &c) {
  sendHeader(c);
  sendPageOpen(c, "Network", "/network");
  c.println(F("<form method='POST' action='/network'>"));
  c.print(F("<label><input type=checkbox name='dhcp'")); if (netDhcp) c.print(F(" checked"));
  c.println(F("> DHCP</label><br><br>"));
  // While DHCP is on, netIp/netMask/etc are just the unused static fallback
  // — show what the interface is *actually* running with instead, so this
  // page reflects reality. These same fields double as the static config
  // once DHCP is unchecked, so leave them editable either way.
  if (netDhcp) {
    c.println(F("<p>DHCP is on — showing the address currently assigned by the network below.</p>"));
    c.print(F("Static IP: <input type=text name='ip' value='")); c.print(Ethernet.localIP()); c.println(F("'><br>"));
    c.print(F("Mask: <input type=text name='mask' value='")); c.print(Ethernet.subnetMask()); c.println(F("'><br>"));
    c.print(F("Gateway: <input type=text name='gw' value='")); c.print(Ethernet.gatewayIP()); c.println(F("'><br>"));
    c.print(F("DNS: <input type=text name='dns' value='")); c.print(Ethernet.dnsServerIP()); c.println(F("'><br>"));
  } else {
    c.print(F("Static IP: <input type=text name='ip' value='")); c.print(netIp); c.println(F("'><br>"));
    c.print(F("Mask: <input type=text name='mask' value='")); c.print(netMask); c.println(F("'><br>"));
    c.print(F("Gateway: <input type=text name='gw' value='")); c.print(netGateway); c.println(F("'><br>"));
    c.print(F("DNS: <input type=text name='dns' value='")); c.print(netDns); c.println(F("'><br>"));
  }
  c.println(F("<button type=submit>Save &amp; Reboot</button></form>"));
  sendPageClose(c);
}
static void handleNetworkPost(const String &body) {
  netDhcp = formHasField(body, "dhcp");
  String v;
  v = formField(body, "ip");   if (v.length()) netIp.fromString(v);
  v = formField(body, "mask"); if (v.length()) netMask.fromString(v);
  v = formField(body, "gw");   if (v.length()) netGateway.fromString(v);
  v = formField(body, "dns");  if (v.length()) netDns.fromString(v);
  saveNetConfig();
  addLog("[WEB] network updated, rebooting");
  g_rebootPending = true;
  g_rebootAt = millis() + 1000;
}

// ================================================================
//  Serial (RS485)
// ================================================================
static const uint32_t kBauds[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

static void sendSerialPage(EthernetClient &c) {
  sendHeader(c);
  sendPageOpen(c, "Serial (RS485)", "/serial");
  c.println(F("<form method='POST' action='/serial'>"));
  c.print(F("Baud: <select name='baud'>"));
  for (int i = 0; i < 13; i++) printOption(c, i, String(kBauds[i]), S0.baudIndex);
  c.println(F("</select><br>"));
  c.print(F("Data bits: <select name='bits'>"));
  printOption(c, 7, "7", S0.dataBits); printOption(c, 8, "8", S0.dataBits);
  c.println(F("</select><br>"));
  c.print(F("Parity: <select name='par'>"));
  printOption(c, 0, "None", S0.parity); printOption(c, 1, "Odd", S0.parity); printOption(c, 2, "Even", S0.parity);
  c.println(F("</select><br>"));
  c.print(F("Stop bits: <select name='stop'>"));
  printOption(c, 1, "1", S0.stopBits); printOption(c, 2, "2", S0.stopBits);
  c.println(F("</select><br>"));
  c.print(F("Pre-delay (us): <input type=number name='pre' value='")); c.print(S0.preDelayUs); c.println(F("'><br>"));
  c.print(F("Post-delay (us): <input type=number name='post' value='")); c.print(S0.postDelayUs); c.println(F("'><br>"));
  c.print(F("Response timeout (ms): <input type=number min=50 max=10000 name='rto' value='"));
  c.print(S0.responseTimeoutMs); c.println(F("'><br>"));
  c.println(F("<p>How long to wait for an RTU slave's reply before giving up. Real field devices usually "
              "answer within tens of ms; PC-based simulators (ModSim/ModScan etc.) can have much larger, "
              "GUI-driven latency spikes — raise this if you're seeing frequent timeout failures against "
              "one of those.</p>"));
  c.println(F("<button type=submit>Save &amp; Apply</button></form>"));
  sendPageClose(c);
}
static void handleSerialPost(const String &body) {
  String v;
  v = formField(body, "baud"); if (v.length()) S0.baudIndex   = (uint8_t)v.toInt();
  v = formField(body, "bits"); if (v.length()) S0.dataBits    = (uint8_t)v.toInt();
  v = formField(body, "par");  if (v.length()) S0.parity      = (uint8_t)v.toInt();
  v = formField(body, "stop"); if (v.length()) S0.stopBits    = (uint8_t)v.toInt();
  v = formField(body, "pre");  if (v.length()) S0.preDelayUs  = (uint16_t)v.toInt();
  v = formField(body, "post"); if (v.length()) S0.postDelayUs = (uint16_t)v.toInt();
  v = formField(body, "rto");  if (v.length()) S0.responseTimeoutMs = (uint16_t)v.toInt();
  if (S0.responseTimeoutMs < 50) S0.responseTimeoutMs = 50;
  saveSerialConfig();
  mbRtuBegin();   // applies live, no reboot needed
  addLog("[WEB] serial settings updated");
}

// ================================================================
//  System
// ================================================================
static void sendSystemPage(EthernetClient &c) {
  sendHeader(c);
  sendPageOpen(c, "System", "/system");
  c.println(F("<form method='POST' action='/system'>"));
  c.print(F("Device name: <input type=text maxlength=31 name='name' value='")); c.print(sysName); c.println(F("'><br>"));
  c.print(F("Login username: <input type=text name='user' value='")); c.print(loginUsername); c.println(F("'><br>"));
  c.println(F("Login password: <input type=password name='pass' value=''><br>"));
  c.print(F("HTTP port: <input type=number name='http' value='")); c.print(httpPort); c.println(F("'><br>"));
  c.print(F("<label><input type=checkbox name='httpen'")); if (httpEnable) c.print(F(" checked"));
  c.println(F("> HTTP server enabled</label><br>"));
  c.println(F("<p>Unchecking this doesn't take effect immediately — the web UI keeps working for "
              "60s after every boot regardless, so a mistaken disable can still be undone before "
              "it actually locks you out (short of a factory reset).</p>"));
  c.print(F("Modbus TCP mode: <select name='mbmode' id='mbmode' style='width:280px' onchange='mbModeToggle()'>"));
  printOption(c, 0, "Server (listen on port)", mbTcpClientMode ? 1 : 0);
  printOption(c, 1, "Client (connect out to host:port)", mbTcpClientMode ? 1 : 0);
  c.println(F("</select><br>"));

  c.print(F("<div id='mbSrv' style='display:")); c.print(mbTcpClientMode ? "none" : "block"); c.println(F("'>"));
  c.print(F("Modbus TCP port: <input type=number name='mbport' value='")); c.print(modbusTcpPort); c.println(F("'><br></div>"));

  c.print(F("<div id='mbCli' style='display:")); c.print(mbTcpClientMode ? "block" : "none"); c.println(F("'>"));
  c.print(F("Remote host: <input type=text name='mbhost' value='")); c.print(mbTcpClientHost); c.println(F("'><br>"));
  c.print(F("Remote port: <input type=number name='mbrport' value='")); c.print(mbTcpClientPort); c.println(F("'><br></div>"));

  c.println(F("<p>Server mode accepts many simultaneous connections (tested well beyond 4). "
              "Client mode dials out to one remote host:port instead — the Modbus protocol "
              "role doesn't change (this device still answers requests), only who opens the "
              "TCP socket. Use Client mode when the remote SCADA/master can't connect to this "
              "device directly (behind a firewall/NAT).</p>"));
  c.println(F("<button type=submit>Save &amp; Reboot</button></form>"));
  c.println(F("<script>function mbModeToggle(){"
              "var c=document.getElementById('mbmode').value=='1';"
              "document.getElementById('mbSrv').style.display=c?'none':'block';"
              "document.getElementById('mbCli').style.display=c?'block':'none';"
              "}</script>"));
  sendPageClose(c);
}
static void handleSystemPost(const String &body) {
  String v;
  v = formField(body, "name");    if (v.length()) sysName = v;
  v = formField(body, "user");    if (v.length()) loginUsername = v;
  v = formField(body, "pass");    if (v.length()) loginPassword = v;
  v = formField(body, "http");    if (v.length()) httpPort = v.toInt();
  httpEnable = formHasField(body, "httpen");
  v = formField(body, "mbport");  if (v.length()) modbusTcpPort = v.toInt();
  v = formField(body, "mbmode");  if (v.length()) mbTcpClientMode = (v.toInt() == 1);
  v = formField(body, "mbhost");  mbTcpClientHost = v;   // may legitimately be cleared
  v = formField(body, "mbrport"); if (v.length()) mbTcpClientPort = v.toInt();
  saveSysConfig();
  addLog("[WEB] system updated, rebooting");
  g_rebootPending = true;
  g_rebootAt = millis() + 1000;
}

// ================================================================
//  Log
// ================================================================
static void sendLogPage(EthernetClient &c) {
  sendHeader(c);
  sendPageOpen(c, "Log", "/log");
  c.print(F("<p>Most recent first — holds the last ")); c.print(MAX_LOGS); c.println(F(" entries.</p>"));
  c.println(F("<pre>"));
  int start = (logCount < MAX_LOGS) ? 0 : logIndex;
  for (int k = logCount - 1; k >= 0; k--) {
    int idx = (start + k) % MAX_LOGS;
    c.println(logs[idx]);
  }
  c.println(F("</pre>"));
  sendPageClose(c);
}

// ================================================================
//  frontendLoop – services exactly one ready client per call
// ================================================================
void frontendLoop() {
  // Disabling the web UI never takes effect immediately — see HTTP_GRACE_MS.
  // Any System-page save reboots the device (resetting millis() to 0), so
  // in practice you always get a fresh 60s window after the reboot that
  // applied the change to undo a mistaken disable before it actually bites.
  if (!httpEnable && millis() > HTTP_GRACE_MS) {
    EthernetClient dead = webServer->available();
    if (dead) dead.stop();
    return;
  }

  EthernetClient c = webServer->available();
  if (!c) return;

  String h; h.reserve(800);
  bool isPost = false;
  int contentLen = 0;
  unsigned long t = millis() + 300;

  while (c.connected() && millis() < t) {
    if (c.available()) {
      char ch = c.read();
      h += ch;
      if (h.endsWith("\r\n\r\n")) {
        if (h.startsWith("POST ")) isPost = true;
        int i = h.indexOf("Content-Length: ");
        if (i >= 0) contentLen = h.substring(i + 16, h.indexOf('\r', i)).toInt();
        break;
      }
      if (h.length() > 2048) break;
    }
  }
  if (h.length() == 0) { c.stop(); return; }

  String body;
  if (isPost && contentLen > 0) {
    body.reserve(contentLen + 8);
    unsigned long tb = millis() + 5000;
    while ((int)body.length() < contentLen && millis() < tb) {
      if (c.available()) { body += (char)c.read(); }
      else { WDT_RESET_COUNTER(); }
    }
  }

  if (h.indexOf("favicon.ico") >= 0) {
    c.println(F("HTTP/1.1 204 No Content\r\n"));
    c.stop();
    return;
  }

  String path = "/";
  int gp = h.indexOf(' ');
  if (gp >= 0) {
    int sp2 = h.indexOf(' ', gp + 1);
    if (sp2 > gp) path = h.substring(gp + 1, sp2);
  }

  // /logout must answer 401 unconditionally — even with valid credentials —
  // since that's the only way to make the browser drop its cached Basic
  // Auth creds (see send401Logout() for why a different realm is used).
  if (path == "/logout") { send401Logout(c); c.stop(); return; }

  bool auth = false;
  int ai = h.indexOf("Authorization: Basic ");
  if (ai >= 0) {
    String a = h.substring(ai + 21, h.indexOf('\r', ai + 21));
    if (a == loginAuthBase64) auth = true;
  }
  if (!auth) { send401(c); c.stop(); return; }

  if (isPost && path == "/points")       { handlePointsPost(body);  sendRedirect(c, "/points");  }
  else if (isPost && path == "/network") { handleNetworkPost(body); sendRedirect(c, "/network"); }
  else if (isPost && path == "/serial")  { handleSerialPost(body);  sendRedirect(c, "/serial");  }
  else if (isPost && path == "/system")  { handleSystemPost(body);  sendRedirect(c, "/system");  }
  else if (path == "/points")  sendPointsPage(c);
  else if (path == "/network") sendNetworkPage(c);
  else if (path == "/serial")  sendSerialPage(c);
  else if (path == "/system")  sendSystemPage(c);
  else if (path == "/log")     sendLogPage(c);
  else sendDashboard(c);

  c.stop();
}
