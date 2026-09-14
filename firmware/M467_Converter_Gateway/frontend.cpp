// ================================================================
//  frontend.cpp  –  embedded web configuration UI (port httpPort)
//
//  Plain server-rendered HTML forms (not an AJAX/JSON SPA) — simplest,
//  most robust option for a C-string-built embedded web server serving
//  a small, fixed number of settings pages. Basic Auth reuses the same
//  base64Encode()/loginAuthBase64 pattern as the reference project.
// ================================================================
#include "frontend.h"

// ================================================================
//  BufferedPrint – batches many small print()/println() calls into a
//  handful of larger network writes.
//
//  EthernetClient::write() (nvtEther) forces an immediate TCP segment on
//  every single call ("Force to send data right now!" — tcp_output() is
//  unconditional, not just when a buffer fills). A page built from many
//  small c.print() fragments turns into many tiny TCP segments instead of
//  a handful of full-size ones: visibly slow, "renders cell by cell" in
//  the browser. Buffering here first and only handing whole chunks to the
//  real client write() fixes that without touching a single call site —
//  every page function already writes to something named "c", so
//  shadowing that name with a BufferedPrint wrapping the real client is
//  enough (see send*Page() below); Print's own print()/println()
//  overloads are built on just write(uint8_t) / write(buf,len), both
//  overridden here.
// ================================================================
class BufferedPrint : public Print {
  public:
    explicit BufferedPrint(EthernetClient &client) : _client(client), _len(0) {}
    ~BufferedPrint() { flush(); }

    virtual size_t write(uint8_t b) {
      if (_len >= sizeof(_buf)) flush();
      _buf[_len++] = b;
      return 1;
    }
    virtual size_t write(const uint8_t *buffer, size_t size) {
      size_t total = size;
      while (size > 0) {
        if (_len >= sizeof(_buf)) flush();
        size_t space = sizeof(_buf) - _len;
        size_t chunk = (size < space) ? size : space;
        memcpy(&_buf[_len], buffer, chunk);
        _len += chunk;
        buffer += chunk;
        size -= chunk;
      }
      return total;
    }
    void flush() {
      if (_len > 0) {
        _client.write(_buf, _len);
        _len = 0;
      }
    }
    using Print::write;

  private:
    EthernetClient &_client;
    uint8_t         _buf[512];
    size_t          _len;
};

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

static void printOption(Print &c, int value, const String &label, int current) {
  c.print("<option value='"); c.print(value); c.print("'");
  if (value == current) c.print(" selected");
  c.print(">"); c.print(label); c.println("</option>");
}

// ================================================================
//  HTTP response helpers
// ================================================================
// No Cache-Control means the browser is free to use its own heuristics to
// decide whether/how long to cache these — which has bitten us before as
// "the page shows old fields/JS even though the firmware was updated".
// Every response this server sends is either state-dependent (config
// reflecting current settings) or a one-shot auth challenge, so none of it
// should ever be cached.
#define NO_CACHE_HEADER "Cache-Control: no-store, no-cache, must-revalidate\r\n"

static void sendHeader(Print &c) {
  c.println(F("HTTP/1.1 200 OK"));
  c.println(F("Connection: close"));
  c.println(F("Content-Type: text/html; charset=utf-8"));
  c.print(F(NO_CACHE_HEADER));
  c.println();
}
static void sendRedirect(EthernetClient &c, const char *location) {
  c.println(F("HTTP/1.1 303 See Other"));
  c.print(F("Location: ")); c.println(location);
  c.println(F("Connection: close"));
  c.print(F(NO_CACHE_HEADER));
  c.println();
}
static void send401(EthernetClient &c) {
  c.println(F("HTTP/1.1 401 Unauthorized"));
  c.println(F("WWW-Authenticate: Basic realm=\"Gateway\""));
  c.println(F("Connection: close"));
  c.println(F("Content-Type: text/html; charset=utf-8"));
  c.print(F(NO_CACHE_HEADER));
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
  c.print(F(NO_CACHE_HEADER));
  c.println();
  c.println(F("<h1>Logged out — close this tab, or enter any credentials to log back in.</h1>"));
}

static void navLink(Print &c, const char *href, const char *label, const char *activePath) {
  c.print(F("<a href='")); c.print(href); c.print(F("'"));
  if (strcmp(href, activePath) == 0) c.print(F(" class='active'"));
  c.print(F(">")); c.print(label); c.print(F("</a>"));
}

static void sendPageOpen(Print &c, const char *title, const char *activePath) {
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
  navLink(c, "/network", "Network",   activePath);
  navLink(c, "/serial",  "Serial",    activePath);
  navLink(c, "/system",  "System",    activePath);
  c.println(F("<a href='/logout' class='logout'>Logout</a></nav>"));
}
static void sendPageClose(Print &c) {
  c.println(F("</body></html>"));
}

// ================================================================
//  Dashboard
// ================================================================
static void sendDashboard(EthernetClient &rawClient) {
  BufferedPrint c(rawClient);
  sendHeader(c);
  sendPageOpen(c, "Dashboard", "/");
  c.print(F("<p>IP: ")); c.print(Ethernet.localIP()); c.println(F("</p>"));
  c.print(F("<p>Uptime: ")); c.print(millis() / 1000); c.println(F(" s</p>"));
  c.print(F("<p>Converter mode: "));
  c.print(converterMode == CONV_TRANSPARENT ? F("transparent") : F("standard tcp&lt;&gt;rtu"));
  c.println(F("</p>"));
  c.print(F("<p>TCP clients connected: ")); c.print(mbConverterActiveClientCount()); c.println(F("</p>"));
  sendPageClose(c);
  c.flush();
}

// ================================================================
//  Network
// ================================================================
static void sendNetworkPage(EthernetClient &rawClient) {
  BufferedPrint c(rawClient);
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
  c.flush();
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

static void sendSerialPage(EthernetClient &rawClient) {
  BufferedPrint c(rawClient);
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
  c.flush();
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
static void sendSystemPage(EthernetClient &rawClient) {
  BufferedPrint c(rawClient);
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

  c.print(F("Converter sub-mode: <select name='convmode' style='width:280px'>"));
  printOption(c, CONV_STANDARD,    "standard (Modbus TCP&lt;-&gt;RTU conversion)", converterMode);
  printOption(c, CONV_TRANSPARENT, "transparent (raw byte tunnel)", converterMode);
  c.println(F("</select><br>"));
  c.print(F("Modbus TCP port: <input type=number name='mbport' value='")); c.print(modbusTcpPort); c.println(F("'><br>"));
  c.println(F("<p>Up to ")); c.print(MAX_TRACKED_CLIENTS);
  c.println(F(" connections, round-robin serviced. Standard: real Modbus TCP (MBAP), CRC "
              "added/verified for you. Transparent: raw byte tunnel — client's bytes ARE the "
              "RTU frame (own CRC), framed by the RTU silent-interval gap on both sides.</p>"));

  c.println(F("<button type=submit>Save &amp; Reboot</button></form>"));
  sendPageClose(c);
  c.flush();
}
static void handleSystemPost(const String &body) {
  String v;
  v = formField(body, "name");     if (v.length()) sysName = v;
  v = formField(body, "user");     if (v.length()) loginUsername = v;
  v = formField(body, "pass");     if (v.length()) loginPassword = v;
  v = formField(body, "http");     if (v.length()) httpPort = v.toInt();
  httpEnable = formHasField(body, "httpen");
  v = formField(body, "mbport");   if (v.length()) modbusTcpPort = v.toInt();
  v = formField(body, "convmode"); if (v.length()) converterMode = (uint8_t)v.toInt();
  saveSysConfig();
  addLog("[WEB] system updated, rebooting");
  g_rebootPending = true;
  g_rebootAt = millis() + 1000;
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

  WDT_MARK("fe:available");
  EthernetClient c = webServer->available();
  if (!c) return;

  String h; h.reserve(800);
  bool isPost = false;
  int contentLen = 0;
  unsigned long t = millis() + 300;

  WDT_MARK("fe:readHeader");
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
    WDT_MARK("fe:readBody");
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

  if (isPost && path == "/network")     { WDT_MARK("fe:postNetwork"); handleNetworkPost(body); sendRedirect(c, "/network"); }
  else if (isPost && path == "/serial") { WDT_MARK("fe:postSerial");  handleSerialPost(body);  sendRedirect(c, "/serial");  }
  else if (isPost && path == "/system") { WDT_MARK("fe:postSystem");  handleSystemPost(body);  sendRedirect(c, "/system");  }
  else if (path == "/network")          { WDT_MARK("fe:sendNetwork"); sendNetworkPage(c); }
  else if (path == "/serial")           { WDT_MARK("fe:sendSerial");  sendSerialPage(c); }
  else if (path == "/system")           { WDT_MARK("fe:sendSystem");  sendSystemPage(c); }
  else                                  { WDT_MARK("fe:sendDashboard"); sendDashboard(c); }

  WDT_MARK("fe:stop");
  c.stop();
}
