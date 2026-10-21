document.addEventListener("DOMContentLoaded", () => {
    // Componentes del DOM
    const selectPuertos = document.getElementById("puertos");
    const selectBaudrate = document.getElementById("baudrate");
    const btnConectar = document.getElementById("conectar");
    const indicadorConexion = document.getElementById("indicador-conexion");
    const textoEstadoConexion = document.getElementById("texto-estado-conexion");

    // D-Pad
    const btnAvanzar = document.getElementById("btn-avanzar");
    const btnRetroceder = document.getElementById("btn-retroceder");
    const btnIzquierda = document.getElementById("btn-izquierda");
    const btnDerecha = document.getElementById("btn-derecha");

    // Entradas
    const inputX = document.getElementById("coord-x");
    const inputY = document.getElementById("coord-y");
    const btnMoverCoords = document.getElementById("btn-mover-coordenadas");

    const inputQ1 = document.getElementById("grados-m1");
    const inputQ2 = document.getElementById("grados-m2");
    const btnMoverGrados = document.getElementById("btn-mover-grados");

    // Servo Z
    const btnZArriba = document.getElementById("btn-z-arriba");
    const btnZAbajo = document.getElementById("btn-z-abajo");
    let estadoZ = 0; // 0: Arriba, 1: Abajo

    const sliderVelocidad = document.getElementById("velocidad");
    const outputVelocidad = document.getElementById("valor-velocidad");
    const btnStop = document.getElementById("stop");

    const visorTrama = document.getElementById("visor-trama");
    const estadoChecksum = document.getElementById("estado-checksum");
    const consolaLog = document.getElementById("consola-log");

    let conectado = false;
    let ultimoLogId = 0;
    let logInterval = null;

    // --- FUNCIÓN PARA DETECTAR Y MOSTRAR ÚNICAMENTE LOS PUERTOS COM ACTIVOS ---
    async function cargarPuertos() {
        try {
            const res = await fetch("/api/puertos");
            const puertos = await res.json();
            selectPuertos.innerHTML = "";

            if (!puertos || puertos.length === 0) {
                const opt = document.createElement("option");
                opt.value = "";
                opt.textContent = "No hay puertos detectados";
                selectPuertos.appendChild(opt);
                btnConectar.disabled = true;
                return;
            }

            btnConectar.disabled = false;
            puertos.forEach(puerto => {
                const opt = document.createElement("option");
                opt.value = puerto;
                opt.textContent = puerto;
                selectPuertos.appendChild(opt);
            });
        } catch (err) {
            console.error("Error al obtener los puertos serie:", err);
            selectPuertos.innerHTML = '<option value="">Error al buscar puertos</option>';
            btnConectar.disabled = true;
        }
    }

    // Cargar puertos inmediatamente al iniciar la página
    cargarPuertos();

    sliderVelocidad.addEventListener("input", (e) => {
        outputVelocidad.textContent = e.target.value;
    });

    // Conexión / Desconexión
    btnConectar.addEventListener("click", async () => {
        if (!conectado) {
            const puerto = selectPuertos.value;
            const baudrate = selectBaudrate.value;

            if (!puerto) {
                alert("Por favor selecciona un puerto serial válido.");
                return;
            }

            try {
                const res = await fetch("/api/conectar", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ puerto, baudrate })
                });

                const data = await res.json();
                if (res.ok) {
                    conectado = true;
                    btnConectar.textContent = "Desconectar";
                    btnConectar.classList.add("conectado");
                    indicadorConexion.style.backgroundColor = "#4EED97";
                    textoEstadoConexion.textContent = "Conectado";
                    iniciarLogs();
                } else {
                    alert("Error al conectar: " + (data.error || data.message));
                }
            } catch (err) {
                alert("No se pudo conectar al servidor Flask.");
            }
        } else {
            try {
                await fetch("/api/desconectar", { method: "POST" });
            } finally {
                conectado = false;
                btnConectar.textContent = "Conectar";
                btnConectar.classList.remove("conectado");
                indicadorConexion.style.backgroundColor = "#b42318";
                textoEstadoConexion.textContent = "Desconectado";
                detenerLogs();
                cargarPuertos(); // Volver a escanear puertos al desconectarse
            }
        }
    });

    // 1. Mover por Coordenadas Cartesianas (X, Y)
    btnMoverCoords.addEventListener("click", async () => {
        const x = parseFloat(inputX.value);
        const y = parseFloat(inputY.value);

        try {
            const res = await fetch("/api/mover", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ x, y, z: estadoZ })
            });
            const data = await res.json();
            if (data.trama) {
                visorTrama.textContent = data.trama;
                estadoChecksum.textContent = "CHECKSUM OK";
                estadoChecksum.style.color = "#4EED97";
            }
        } catch (err) {
            console.error("Error al enviar coordenada:", err);
        }
    });

    // 2. Mover por Ángulos Directos (q1, q2)
    btnMoverGrados.addEventListener("click", async () => {
        const q1 = parseFloat(inputQ1.value);
        const q2 = parseFloat(inputQ2.value);

        try {
            const res = await fetch("/api/angulos", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ q1, q2, z: estadoZ })
            });
            const data = await res.json();
            if (data.trama) {
                visorTrama.textContent = data.trama;
                estadoChecksum.textContent = "CHECKSUM OK";
                estadoChecksum.style.color = "#4EED97";
            }
        } catch (err) {
            console.error("Error al enviar ángulos:", err);
        }
    });

    // 3. Control Servo Z (0 = Arriba, 1 = Abajo)
    async function cambiarZ(nuevoZ) {
        estadoZ = nuevoZ;
        try {
            const res = await fetch("/api/servo_z", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ z: estadoZ })
            });
            const data = await res.json();
            if (data.trama) visorTrama.textContent = data.trama;
        } catch (err) {
            console.error("Error en Servo Z:", err);
        }
    }

    btnZArriba.addEventListener("click", () => cambiarZ(0));
    btnZAbajo.addEventListener("click", () => cambiarZ(1));

    // 4. Jogging (Flechas D-Pad)
    async function enviarJog(dir) {
        try {
            const res = await fetch("/api/jog", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ dir })
            });
            const data = await res.json();
            if (data.trama) visorTrama.textContent = data.trama;
        } catch (err) {
            console.error("Error Jog:", err);
        }
    }

    btnAvanzar.addEventListener("click", () => enviarJog("avanzar"));
    btnRetroceder.addEventListener("click", () => enviarJog("retroceder"));
    btnIzquierda.addEventListener("click", () => enviarJog("izquierda"));
    btnDerecha.addEventListener("click", () => enviarJog("derecha"));

    // Emergency Stop
    btnStop.addEventListener("click", async () => {
        try {
            const res = await fetch("/api/emergencia", { method: "POST" });
            const data = await res.json();
            visorTrama.textContent = "PARADA EMERGENCIA";
        } catch (err) {
            console.error("Error en parada de emergencia:", err);
        }
    });

    // Diagnósticos / Logs Polling
    function iniciarLogs() {
        if (!logInterval) logInterval = setInterval(obtenerLogs, 400);
    }

    function detenerLogs() {
        if (logInterval) {
            clearInterval(logInterval);
            logInterval = null;
        }
    }

    async function obtenerLogs() {
        try {
            const res = await fetch(`/api/log?desde=${ultimoLogId}`);
            const logs = await res.json();
            logs.forEach(([id, texto]) => {
                ultimoLogId = id;
                const p = document.createElement("p");
                p.textContent = texto;
                consolaLog.appendChild(p);
            });
            if (logs.length > 0) consolaLog.scrollTop = consolaLog.scrollHeight;
        } catch (err) {
            console.error("Error en lectura de logs:", err);
        }
    }
});