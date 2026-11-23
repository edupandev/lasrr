const http = require('http');
const fs = require('fs');
const path = require('path');
const { exec } = require('child_process');

const PORT = process.env.PORT || 3333;
const PUBLIC_DIR = __dirname;
const TELEMETRY_FILE = path.join(PUBLIC_DIR, 'telemetry.jsonl');

const MIME_TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.svg': 'image/svg+xml'
};

const server = http.createServer((req, res) => {
  // CORS Headers
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

  if (req.method === 'OPTIONS') {
    res.writeHead(204);
    res.end();
    return;
  }

  let reqPath = req.url.split('?')[0];

  // API de Telemetria em Tempo Real
  if (req.method === 'POST' && reqPath === '/api/telemetry') {
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => {
      try {
        const payload = JSON.parse(body || '{}');
        payload.serverIso = new Date().toISOString();
        const line = JSON.stringify(payload) + '\n';
        fs.appendFileSync(TELEMETRY_FILE, line, 'utf8');

        // Mostra no terminal eventos relevantes
        if (payload.type === 'SPEED_PASS') {
          console.log(`[PASSAGEM] ${payload.data.v_km_h} km/h | Δt: ${payload.data.deltaMs}ms | Sentido: ${payload.data.dir} | Δs: ${payload.data.trapDist}m`);
        } else if (payload.type === 'BEAM_TRIGGER') {
          console.log(`[DISPARO] Feixe ${payload.data.sensor} acionado! Tipo: ${payload.data.triggerType} (Dist: ${payload.data.dist}m, Valid: ${payload.data.isValid})`);
        } else if (payload.type === 'BG_CAPTURED') {
          console.log(`[FUNDO] Feixe ${payload.data.sensor} calibrado: ${payload.data.bgDist}m (Corte: < ${payload.data.cutoff}m)`);
        }

        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: true }));
      } catch (err) {
        res.writeHead(400, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: err.message }));
      }
    });
    return;
  }

  // Rota de leitura dos últimos registros de telemetria
  if (req.method === 'GET' && reqPath === '/api/telemetry') {
    if (!fs.existsSync(TELEMETRY_FILE)) {
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end('[]');
      return;
    }
    const lines = fs.readFileSync(TELEMETRY_FILE, 'utf8').trim().split('\n').filter(Boolean);
    const recent = lines.slice(-50).map(l => {
      try { return JSON.parse(l); } catch(e) { return null; }
    }).filter(Boolean);
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(recent));
    return;
  }

  // Limpeza de telemetria
  if (req.method === 'POST' && reqPath === '/api/telemetry/clear') {
    try {
      if (fs.existsSync(TELEMETRY_FILE)) fs.unlinkSync(TELEMETRY_FILE);
      console.log('[TELEMETRY] Log de telemetria reiniciado.');
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ok: true }));
    } catch (e) {
      res.writeHead(500); res.end(e.message);
    }
    return;
  }

  // Arquivos estáticos
  if (reqPath === '/' || reqPath === '') {
    reqPath = '/index.html';
  }

  const filePath = path.join(PUBLIC_DIR, reqPath);

  // Prevent path traversal
  if (!filePath.startsWith(PUBLIC_DIR)) {
    res.writeHead(403);
    res.end('Acesso Negado');
    return;
  }

  fs.stat(filePath, (err, stats) => {
    if (err || !stats.isFile()) {
      res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
      res.end('Arquivo não encontrado');
      return;
    }

    const ext = path.extname(filePath).toLowerCase();
    const contentType = MIME_TYPES[ext] || 'application/octet-stream';

    res.writeHead(200, { 'Content-Type': contentType });
    fs.createReadStream(filePath).pipe(res);
  });
});

server.listen(PORT, '127.0.0.1', () => {
  const url = `http://localhost:${PORT}`;
  console.log('===========================================================');
  console.log(`[Visualizador Serial Meskernel PTFH com Telemetria Ativa]`);
  console.log(`Servidor rodando em: ${url}`);
  console.log(`Logs em tempo real salvos em: ${TELEMETRY_FILE}`);
  console.log('===========================================================');

  // Abre automaticamente no navegador padrão do Windows se não for daemon reexecutado
  if (!process.env.NO_AUTO_OPEN) {
    exec(`start ${url}`);
  }
});
