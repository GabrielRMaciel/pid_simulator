/**
 * Implementa um controlador PID posicional com filtro derivativo e anti-windup.
 * Baseado nos requisitos do PRD[cite: 34, 96].
 */
export class PIDController {
    // Ganhos do controlador
    _Kp = 1;
    _Ki = 0;
    _Kd = 0;

    // Termos internos para o cálculo
    _proportionalTerm = 0;
    _integralTerm = 0;
    _derivativeTerm = 0;

    // Memória para o cálculo
    _lastError = 0;           // Erro anterior para o termo derivativo
    _lastProcessVariable = 0; // PV anterior para o filtro derivativo

    // Limites e Anti-Windup
    _outputMin = -100;
    _outputMax = 100;

    // Filtro Derivativo (para reduzir ruído) 
    _alpha = 0.1; // Fator de filtro (ex: 0.1). Um valor menor resulta em mais filtragem.

    /**
     * @param {number} Kp Ganho Proporcional
     * @param {number} Ki Ganho Integral
     * @param {number} Kd Ganho Derivativo
     * @param {object} options Opções adicionais como limites e alpha
     */
    constructor(Kp, Ki, Kd, options = {}) {
        this.setGains(Kp, Ki, Kd);
        this._outputMin = options.outputMin ?? this._outputMin;
        this._outputMax = options.outputMax ?? this._outputMax;
        this._alpha = options.alpha ?? this._alpha;
    }

    /**
     * Atualiza os ganhos do controlador.
     */
    setGains(Kp, Ki, Kd) {
        this._Kp = Kp;
        this._Ki = Ki;
        this._Kd = Kd;
    }

    /**
     * Calcula a saída do controlador (Variável Manipulada - MV).
     * @param {number} setpoint O valor desejado (SP).
     * @param {number} processVariable O valor medido do processo (PV).
     * @param {number} deltaTime O tempo, em segundos, desde a última atualização.
     * @returns {number} A saída do controlador (MV) saturada entre os limites.
     */
    update(setpoint, processVariable, deltaTime) {
        if (deltaTime <= 0) return this._proportionalTerm + this._integralTerm + this._derivativeTerm;

        const error = setpoint - processVariable;

        // --- Termo Proporcional (P) ---
        this._proportionalTerm = this._Kp * error;

        // --- Termo Derivativo (D) com filtro [cite: 38, 96] ---
        // Usamos a derivada da variável de processo (PV) em vez da derivada do erro
        // para evitar "derivative kick" quando o setpoint muda abruptamente.
        const pvDerivative = (processVariable - this._lastProcessVariable) / deltaTime;
        // Aplica o filtro passa-baixa no termo derivativo
        this._derivativeTerm = this._derivativeTerm * (1 - this._alpha) - (this._Kd * pvDerivative * this._alpha);


        // --- Termo Integral (I) com Anti-Windup  ---
        // A lógica de anti-windup acontece aqui. Só integramos se a saída
        // não estiver saturada, evitando que o termo integral "exploda".
        const potentialOutput = this._proportionalTerm + this._integralTerm + this._derivativeTerm;
        if (potentialOutput < this._outputMax && potentialOutput > this._outputMin) {
            this._integralTerm += this._Ki * error * deltaTime;
        }

        // --- Soma e Saturação da Saída ---
        let output = this._proportionalTerm + this._integralTerm + this._derivativeTerm;
        output = Math.max(this._outputMin, Math.min(this._outputMax, output));

        // Armazena os valores para a próxima iteração
        this._lastError = error;
        this._lastProcessVariable = processVariable;

        return output;
    }

    /**
     * Reseta o estado do controlador (termos integrais e erros anteriores).
     * Útil para "Bumpless Transfer"  ou ao reiniciar a simulação.
     */
    reset() {
        this._integralTerm = 0;
        this._lastError = 0;
        this._lastProcessVariable = 0;
        this._derivativeTerm = 0;
    }
}