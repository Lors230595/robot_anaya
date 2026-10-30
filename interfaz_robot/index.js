const API_URL = "http://127.0.0.1:5000/api";

document.addEventListener("DOMContentLoaded", () => {
    cargarPuertos();
    configurarEventos();
});

// 1. Obtener la lista de puertos serie desde Flask
async function cargarPuertos() {
    const selectPuertos = document.getElementById("puertos");
    if (!selectPuertos) return;

    try {
        const res = await fetch(`${API_URL}/puertos`);
        if (!res.ok) throw new Error(`HTTP status: ${res.status}`);
        
        const data = await res.json();
        selectPuertos.innerHTML = "";
        
        data.puertos.forEach(p => {
            const option = document.createElement("option");
            option.value = p.puerto;
            option.textContent = p.label;
            selectPuertos.appendChild(option);
        });
    } catch (err) {
        logConsola("Error al buscar puertos: Asegúrate de que 'python App.py' está corriendo.", "error");
    }
}

// 2. Configurar todos los escuchadores de eventos
function configurarEventos() {
    // Conexión
    document.getElementById("conectar")?.addEventListener("click", conectarESP);

    // MODO 1: CRUCETA D-PAD ESTRELLA (Pasos relativos de ±0.25 cm / 2.5 mm)
    // Funciona en conjunto con las reglas CSS 'pointer-events'
    document.getElementById("btn-avanzar")?.addEventListener("click", () => {
        enviarComando({ tipo: "STEP_Y", dy: 0.25 });
    });

    document.getElementById("btn-retroceder")?.addEventListener("click", () => {
        enviarComando({ tipo: "STEP_Y", dy: -0.25 });
    });

    document.getElementById("btn-derecha")?.addEventListener("click", () => {
        enviarComando({ tipo: "STEP_X", dx: 0.25 });
    });

    document.getElementById("btn-izquierda")?.addEventListener("click", () => {
        enviarComando({ tipo: "STEP_X", dx: -0.25 });
    });

    document.getElementById("btn-detener")?.addEventListener("click", () => {
        enviarComando({ tipo: "STOP" });
    });

    // MODO 2: MOVER POR COORDENADAS (X, Y)
    document.getElementById("btn-mover-coordenadas")?.addEventListener("click", () => {
        const xInput = document.getElementById("coord-x");
        const yInput = document.getElementById("coord-y");

        const xVal = parseFloat(xInput.value);
        const yVal = parseFloat(yInput.value);

        if (isNaN(xVal) || isNaN(yVal)) {
            logConsola("Ingresa coordenadas válidas para X e Y.", "error");
            return;
        }

        if (xVal < -17 || xVal > 17 || yVal < -17 || yVal > 17) {
            logConsola("Coordenadas fuera del límite máximo del área de trabajo (-17 a 17 cm).", "error");
            return;
        }

        enviarComando({ tipo: "MOVE_XY", x: xVal, y: yVal });

        // Limpiar los inputs
        xInput.value = "";
        yInput.value = "";
    });

    // MODO 3: MOVER POR ÁNGULOS (Motor 1, Motor 2)
    document.getElementById("btn-mover-grados")?.addEventListener("click", () => {
        const m1Input = document.getElementById("grados-m1");
        const m2Input = document.getElementById("grados-m2");

        const q1Val = parseFloat(m1Input.value);
        const q2Val = parseFloat(m2Input.value);

        if (isNaN(q1Val) || isNaN(q2Val)) {
            logConsola("Ingresa ángulos válidos para Motor 1 y Motor 2.", "error");
            return;
        }

        enviarComando({ tipo: "MOVE_ANG", q1: q1Val, q2: q2Val });

        // Limpiar los inputs
        m1Input.value = "";
        m2Input.value = "";
    });

    // CONTROL DEL SERVO EJE Z
    document.getElementById("btn-z-arriba")?.addEventListener("click", () => {
        enviarComando({ tipo: "SERVO_Z", z: 0 });
    });

    document.getElementById("btn-z-abajo")?.addEventListener("click", () => {
        enviarComando({ tipo: "SERVO_Z", z: 1 });
    });

    // BOTONES DE SEGURIDAD Y ORIGEN
    document.getElementById("stop")?.addEventListener("click", () => {
        enviarComando({ tipo: "STOP" });
    });

    document.getElementById("btn-home")?.addEventListener("click", () => {
        enviarComando({ tipo: "HOME" });
    });
}

// 3. Conexión Serial Simulada / Física
async function conectarESP() {
    const puerto = document.getElementById("puertos")?.value;
    const baudrate = document.getElementById("baudrate")?.value;

    try {
        const res = await fetch(`${API_URL}/conectar`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ puerto, baudrate })
        });
        const data = await res.json();

        if (res.ok) {
            actualizarEstadoConexion(true, puerto);
            logConsola(data.mensaje, "ok");
        } else {
            actualizarEstadoConexion(false, puerto);
            logConsola(data.mensaje, "error");
        }
    } catch (err) {
        logConsola("Error de red al intentar conectar con el servidor.", "error");
    }
}

// 4. Enviar comandos al backend y actualizar interfaz
async function enviarComando(payload) {
    try {
        const res = await fetch(`${API_URL}/comando`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload)
        });
        const data = await res.json();

        if (res.ok) {
            document.getElementById("visor-trama").textContent = data.trama;
            document.getElementById("estado-checksum").textContent = `CHECKSUM: ${data.checksum}`;
            logConsola(data.log, "info");
        } else {
            // Notifica en consola si el movimiento no es físicamente alcanzable
            logConsola(data.mensaje, "error");
        }
    } catch (err) {
        logConsola("Error de comunicación con la API.", "error");
    }
}

// 5. Actualización visual de estado
function actualizarEstadoConexion(conectado, puerto) {
    const indicador = document.getElementById("indicador-conexion");
    const texto = document.getElementById("texto-estado-conexion");

    if (indicador) indicador.style.backgroundColor = conectado ? "#2e7d32" : "#b42318";
    if (texto) texto.textContent = conectado ? `Conectado a ${puerto}` : "Desconectado";
}

// 6. Imprimir en la Consola de Diagnóstico
function logConsola(mensaje, tipo = "info") {
    const consola = document.getElementById("consola-log");
    if (!consola) return;

    const linea = document.createElement("div");
    linea.style.fontSize = "0.82rem";
    linea.style.marginBottom = "4px";

    if (tipo === "error") linea.style.color = "#ff6b6b";
    else if (tipo === "ok") linea.style.color = "#51cf66";
    else linea.style.color = "#F0D79A";

    linea.textContent = `> ${mensaje}`;
    consola.appendChild(linea);
    consola.scrollTop = consola.scrollHeight;
}