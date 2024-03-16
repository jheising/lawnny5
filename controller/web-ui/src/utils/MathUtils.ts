export class MathUtils {
    static expoFunction(x: number, expoFactor: number): number {
        return expoFactor * Math.pow(x, 3) + (1 - expoFactor) * x;
    }
}