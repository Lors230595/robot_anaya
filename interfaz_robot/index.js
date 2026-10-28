document.addEventListener('DOMContentLoaded', () => {
    // Configuración de la URL del servidor Flask
    const FLASK_URL = 'http://127.0.0.1:5000';

    // Elementos del DOM - Conexión Serial
    const selectPuertos = document.getElementById('puertos');
    const selectBaudrate = document.getElementById('baudrate');
    const btnConectar = document.getElementById('conectar');
    const indicadorConexion = document.getElementById('indicador-conexion');
    const textoEstadoConexion = document.getElementById('texto-estado-conexion');

    // Elementos del DOM - Controles de Movimiento
    const inputCoordX = document.getElementById('coord-x');
    const inputCoordY = document.getElementById('coord-y');
    const btnMoverCoordenadas = document.getElementById('btn-mover-coordenadas');

    const inputGradosM1 = document.getElementById('grados-m1');
    const inputGradosM2 = document.getElementById('grados-m2');
    const btnMoverGrados = document.getElementById('btn-mover-grados');

    const btnHome = document.getElementById('btn-home');
    const sliderVelocidad = document.getElementById('velocidad');
    const outputVelocidad = document.getElementById('valor-velocidad');

    // Elementos del DOM - Cruceta (D-Pad)
    const btnAvanzar = document.getElementById('btn-avanzar');
    const btnRetroceder = document.getElementById('btn-retroceder');
    const btnIzquierda = document.getElementById('btn-izquierda');
    const btnDerecha = document.getElementById('btn-derecha');
    const btnDetener = document.getElementById('btn-detener');

    // Elementos del DOM - Servo, Stop y Diagnóstico
    const btnZArriba = document.getElementById('btn-z-arriba');
    const btnZAbajo = document.getElementById('btn-z-abajo');
    const btnStop = document.getElementById('stop');
    const visorTrama = document.getElementById('visor-trama');
    const estadoChecksum = document.getElementById('estado-checksum');
    const consolaLog = document.getElementById('consola-log');

    let estaConectado = false;

    // =============================================================
    // SISTEMA DE ALERTAS FLOTANTES (WARNINGS / BANNERS)
    // =============================================================
    function mostrarAlerta(mensaje, tipo = 'warning') {
        let contenedor = document.getElementById('contenedor-alertas');
        if (!contenedor) {
            contenedor = document.createElement('div');
            contenedor.id = 'contenedor-alertas';
            Object.assign(contenedor.style, {
                position: 'fixed',
                top: '20px',
                left: '50%',
                transform: 'translateX(-50%)',
                zIndex: '9999',
                display: 'flex',
                flexDirection: 'column',
                gap: '10px',
                maxWidth: '90%',
                width: '400px'
            });
            document.body.appendChild(contenedor);
        }

        const alerta = document.createElement('div');
        const colorBorde = tipo === 'warning' ? '#F0D79A' : (tipo === 'error' ? '#FF6B6B' : '#4EED97');
        const colorFondo = tipo === 'warning' ? 'rgba(30, 25, 10, 0.95)' : (tipo === 'error' ? 'rgba(40, 10, 10, 0.95)' : 'rgba(10, 30, 15, 0.95)');

        Object.assign(alerta.style, {
            background: colorFondo,
            color: '#F8F6EE',
            border: `1px solid ${colorBorde}`,
            borderRadius: '8px',
            padding: '12px 16px',
            boxShadow: '0 4px 20px rgba(0,0,0,0.5)',
            fontFamily: "'Montserrat', sans-serif",
            fontSize: '0.85rem',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            backdropFilter: 'blur(6px)',
            opacity: '0',
            transition: 'all 0.3s ease'
        });

        alerta.innerHTML = `
            <span>⚠️ <strong>${tipo.toUpperCase()}:</strong> ${mensaje}</span>
            <button style="background:none; border:none; color:#F0D79A; font-size:1.1rem; cursor:pointer; margin-left:10px;">&times;</button>
        `;

        const btnCerrar = alerta.querySelector('button');
        btnCerrar.onclick = () => {
            alerta.style.opacity = '0';
            setTimeout(() => alerta.remove(), 300);
        };

        contenedor.appendChild(alerta);
        requestAnimationFrame(() => {
            alerta.style.opacity = '1';
        });

        setTimeout(() => {
            if (alerta.parentNode) {
                alerta.style.opacity = '0';
                setTimeout(() => alerta.remove(), 300);
            }
        }, 4000);
    }

    // =============================================================
    // FUNCIONES DE DIAGNÓSTICO Y TRAMA
    // =============================================================
    function registrarLog(mensaje, tipo = 'TX') {
        const p = document.createElement('p');
        const hora = new Date().toLocaleTimeString();
        p.textContent = `[${hora}] [${tipo}] ${mensaje}`;
        consolaLog.appendChild(p);
        consolaLog.scrollTop = consolaLog.scrollHeight;
    }

    function calcularChecksum(cadena) {
        let checksum = 0;
        for (let i = 0; i < cadena.length; i++) {
            checksum ^= cadena.charCodeAt(i);
        }
        return checksum.toString(16).padStart(2, '0').toUpperCase();
    }

    function construirTrama(comando, parametros = []) {
        const cuerpo = [comando, ...parametros].join(',');
        const checksum = calcularChecksum(cuerpo);
        return `$${cuerpo}*${checksum}\n`;
    }

    // =============================================================
    // ENVÍO DE DATOS A FLASK
    // =============================================================
    async function enviarComando(comando, parametros = [], limpiarInputs = null) {
        if (!estaConectado && comando !== 'CONECTAR' && comando !== 'LISTAR_PUERTOS') {
            mostrarAlerta('No hay conexión serial establecida con el ESP.', 'warning');
            return;
        }

        const trama = construirTrama(comando, parametros);
        visorTrama.textContent = trama.trim();
        estadoChecksum.textContent = 'CHECKSUM: OK';
        registrarLog(trama.trim(), 'TX');

        try {
            const respuesta = await fetch(`${FLASK_URL}/api/comando`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ trama: trama })
            });

            const resultado = await respuesta.json();

            if (respuesta.ok && resultado.status === 'success') {
                if (resultado.rx) {
                    registrarLog(resultado.rx, 'RX');
                }
                // Limpieza de inputs en caso de éxito
                if (limpiarInputs && typeof limpiarInputs === 'function') {
                    limpiarInputs();
                }
            } else {
                mostrarAlerta(resultado.message || 'Error en la respuesta del servidor.', 'error');
            }
        } catch (error) {
            mostrarAlerta('Error de comunicación con el servidor Flask.', 'error');
            registrarLog('Error de red al conectar con Flask', 'ERR');
        }
    }

    // =============================================================
    // GESTIÓN DE PUERTOS Y CONEXIÓN
    // =============================================================
    async function cargarPuertos() {
        try {
            const res = await fetch(`${FLASK_URL}/api/puertos`);
            const datos = await res.json();
            selectPuertos.innerHTML = '';
            
            if (datos.puertos && datos.puertos.length > 0) {
                datos.puertos.forEach(puerto => {
                    const option = document.createElement('option');
                    option.value = puerto;
                    option.textContent = puerto;
                    selectPuertos.appendChild(option);
                });
            } else {
                selectPuertos.innerHTML = '<option value="">Sin puertos disponibles</option>';
            }
        } catch (err) {
            selectPuertos.innerHTML = '<option value="">Error al cargar puertos</option>';
            mostrarAlerta('No se pudo conectar con el servidor Flask para listar puertos.', 'warning');
        }
    }

    btnConectar.addEventListener('click', async () => {
        const puerto = selectPuertos.value;
        const baudrate = selectBaudrate.value;

        if (!puerto) {
            mostrarAlerta('Por favor, selecciona un puerto serial válido.', 'warning');
            return;
        }

        try {
            const res = await fetch(`${FLASK_URL}/api/conexion`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ puerto, baudrate, accion: estaConectado ? 'desconectar' : 'conectar' })
            });
            const data = await res.json();

            if (res.ok && data.status === 'success') {
                estaConectado = !estaConectado;
                if (estaConectado) {
                    btnConectar.textContent = 'Desconectar';
                    btnConectar.classList.add('conectado');
                    indicadorConexion.style.backgroundColor = '#4EED97';
                    textoEstadoConexion.textContent = 'Conectado';
                    registrarLog(`Conectado a ${puerto} a ${baudrate} baudios`, 'SYS');
                } else {
                    btnConectar.textContent = 'Conectar';
                    btnConectar.classList.remove('conectado');
                    indicadorConexion.style.backgroundColor = '#b42318';
                    textoEstadoConexion.textContent = 'Desconectado';
                    registrarLog('Puerto desconectado', 'SYS');
                }
            } else {
                mostrarAlerta(data.message || 'Error al cambiar estado de conexión.', 'error');
            }
        } catch (err) {
            mostrarAlerta('Error al intentar comunicar con el servidor Flask.', 'error');
        }
    });

    // =============================================================
    // EVENTOS DE CONTROL DE MOVIMIENTO
    // =============================================================

    // 1. Coordenadas Cartesianas (X, Y)
    btnMoverCoordenadas.addEventListener('click', () => {
        const xVal = parseFloat(inputCoordX.value);
        const yVal = parseFloat(inputCoordY.value);

        if (isNaN(xVal) || isNaN(yVal)) {
            mostrarAlerta('Ingresa valores numéricos para X e Y.', 'warning');
            return;
        }

        if (xVal < 0 || xVal > 10 || yVal < 0 || yVal > 10) {
            mostrarAlerta('Las coordenadas deben estar en el rango de 0 a 10 cm.', 'warning');
            return;
        }

        const xFix = xVal.toFixed(1);
        const yFix = yVal.toFixed(1);

        enviarComando('MOVE_XY', [xFix, yFix, sliderVelocidad.value], () => {
            inputCoordX.value = '';
            inputCoordY.value = '';
        });
    });

    // 2. Posicionamiento Angular (M1, M2)
    btnMoverGrados.addEventListener('click', () => {
        const m1Val = parseInt(inputGradosM1.value);
        const m2Val = parseInt(inputGradosM2.value);

        if (isNaN(m1Val) || isNaN(m2Val)) {
            mostrarAlerta('Ingresa ángulos válidos para los motores.', 'warning');
            return;
        }

        if (m1Val < -180 || m1Val > 180 || m2Val < -180 || m2Val > 180) {
            mostrarAlerta('Los ángulos deben estar entre -180° y 180°.', 'warning');
            return;
        }

        enviarComando('MOVE_ANG', [m1Val, m2Val, sliderVelocidad.value], () => {
            inputGradosM1.value = '';
            inputGradosM2.value = '';
        });
    });

    // 3. Posición de Origen (Home)
    btnHome.addEventListener('click', () => {
        enviarComando('HOME', [sliderVelocidad.value]);
    });

    // 4. Cruceta D-Pad
    btnAvanzar.addEventListener('click', () => enviarComando('JOG', ['Y+', sliderVelocidad.value]));
    btnRetroceder.addEventListener('click', () => enviarComando('JOG', ['Y-', sliderVelocidad.value]));
    btnIzquierda.addEventListener('click', () => enviarComando('JOG', ['X-', sliderVelocidad.value]));
    btnDerecha.addEventListener('click', () => enviarComando('JOG', ['X+', sliderVelocidad.value]));
    btnDetener.addEventListener('click', () => enviarComando('STOP', ['0']));

    // 5. Control de Servo Z
    btnZArriba.addEventListener('click', () => enviarComando('SERVO_Z', ['0']));
    btnZAbajo.addEventListener('click', () => enviarComando('SERVO_Z', ['1']));

    // 6. Parada de Emergencia
    btnStop.addEventListener('click', () => {
        enviarComando('EMERGENCY_STOP', ['1']);
        mostrarAlerta('¡PARADA DE EMERGENCIA ACTIVADA!', 'error');
    });

    // 7. Slider de Velocidad
    sliderVelocidad.addEventListener('input', (e) => {
        outputVelocidad.value = e.target.value;
    });

    // Carga inicial de puertos
    cargarPuertos();
});