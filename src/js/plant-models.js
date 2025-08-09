export class MechanicalSystem {
    // Parâmetros Físicos
    _inertia; // Equivalente à massa do objeto
    _friction; // Coeficiente de atrito
    _load; // Força constante atuando no sistema (ex: gravidade)

    // Variáveis de Estado
    state = {
        position: 0,
        velocity: 0
    };

    // Perturbações externas
    _disturbance = 0;

    /**
     * @param {object} params Parâmetros de configuração da planta
     * @param {number} params.inertia Inércia do sistema
     * @param {number} params.friction Coeficiente de atrito
     * @param {number} params.load Força de carga constante
     * @param {number} params.initialPosition Posição inicial opcional
     * @param {boolean} params.autoEquilibrium Se true, ajusta posição inicial para equilíbrio
     */
    constructor(params) {
        this._inertia = params.inertia;
        this._friction = params.friction;
        this._load = params.load;

        if (params.autoEquilibrium) {
            // Equilíbrio estático: posição inicial tal que a força de carga é balanceada pela saída 0
            // Como não há mola ou outra força restauradora, o equilíbrio é simplesmente manter velocidade zero
            // Neste caso, a posição inicial não importa fisicamente, mas definimos um valor útil:
            this.state.position = params.initialPosition ?? 0;
        } else {
            this.state.position = params.initialPosition || 0;
        }
    }

    update(controlOutput, dt) {
        const frictionForce = -this._friction * this.state.velocity;
        const loadForce = -this._load;
        const netForce = controlOutput + this._disturbance + frictionForce + loadForce;
        const acceleration = netForce / this._inertia;

        this.state.velocity += acceleration * dt;
        this.state.position += this.state.velocity * dt;

        return this.state;
    }

    setDisturbance(value) {
        this._disturbance = value;
    }

    reset() {
        this.state.velocity = 0;
        this._disturbance = 0;
    }
}
