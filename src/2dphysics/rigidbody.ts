import { PointCal, Point } from "point2point";

export interface RigidBody {

    step(deltaTime: number): void;
    isStatic(): boolean;
    isMovingStatic(): boolean;
    getMinMaxProjection(unitvector: Point): {min: number, max: number};
    getCollisionAxes(relativeBody: RigidBody): Point[];
    applyForce(force: Point): void;
    applyForceInOrientation(force: Point): void;
    getAABB(): {min: Point, max: Point};
    getMass(): number;
    getLinearVelocity(): Point;
    setLinearVelocity(linearVelocity: Point): void;
    move(delta: Point): void;

}

export abstract class BaseRigidBody implements RigidBody{
    
    protected center: Point;
    protected orientationAngle: number = 0;
    protected mass: number = 50;
    protected linearVelocity: Point;
    protected linearAcceleartion: Point;
    protected force: Point;
    protected isStaticBody: boolean = false;
    protected staticFrictionCoeff: number = 0.3;
    protected dynamicFrictionCoeff: number = 0.3;
    protected frictionEnabled: boolean = false;
    protected isMovingStaticBody: boolean = false;
    protected angularVelocity: number; // in radians
    

    constructor(center: Point, orientationAngle: number = 0, mass: number = 50, isStaticBody: boolean = false, frictionEnabled: boolean = false){
        this.center = center;
        this.orientationAngle = orientationAngle;
        this.mass = mass;
        this.isStaticBody = isStaticBody;
        this.frictionEnabled = frictionEnabled;
        this.force = {x: 0, y: 0};
        this.linearAcceleartion = {x: 0, y: 0};
        this.linearVelocity = {x: 0, y: 0};
        this.angularVelocity = 0;
    }

    move(delta: Point): void {
        if (!this.isStatic()){
            this.center = PointCal.addVector(this.center, delta);
        }
    }

    rotateRadians(angle: number): void {
        this.orientationAngle += angle;
    }

    getCenter(): Point {
        return this.center;
    }
    
    getOrientationAngle(): number{
        return this.orientationAngle;
    }

    getMass(): number{
        return this.mass;
    }

    getLinearVelocity(): Point{
        return this.linearVelocity;
    }

    getAngularVelocity(): number{
        return this.angularVelocity;
    }

    isStatic(): boolean{
        return this.isStaticBody;
    }

    isMovingStatic(): boolean {
        return this.isMovingStaticBody;
    }

    setLinearVelocity(linearVelocity: Point): void {
        this.linearVelocity = linearVelocity;
    }

    setMovingStatic(movingStatic: boolean):void {
        this.isMovingStaticBody = movingStatic;
    }

    setOrientationAngle(angle: number): void{
        this.orientationAngle = angle;
    }

    setAngularVelocity(angularVelocity: number): void{
        this.angularVelocity = angularVelocity;
    }

    applyForce(force: Point): void {
        if (PointCal.magnitude(this.force) !== 0){
            this.force = PointCal.addVector(this.force, force);
        } else {
            this.force = force;
        }
    }

    applyForceInOrientation(force: Point | number): void {
        let forceTransformed: Point;
        if (typeof force === "number") {
            forceTransformed = PointCal.rotatePoint({x: force, y: 0}, this.orientationAngle);
        } else {
            forceTransformed = PointCal.rotatePoint(force, this.orientationAngle);
        }
        this.applyForce(forceTransformed);
    }

    step(deltaTime: number): void {
        if (this.frictionEnabled) {
            if (this.isStatic()  || 
                (this.linearVelocity.x == 0 && 
                 this.linearVelocity.y == 0 && 
                 PointCal.magnitude(PointCal.subVector(this.force, {x: 0, y: 0})) >= 0 && 
                 PointCal.magnitude(this.force) < this.staticFrictionCoeff * this.mass * 9.81)
                ) {
                this.force = {x: 0, y: 0};
                return;
            }
            let kineticFrictionDirection = PointCal.multiplyVectorByScalar(PointCal.unitVector(this.linearVelocity), -1);
            let kineticFriction = PointCal.multiplyVectorByScalar(kineticFrictionDirection, this.dynamicFrictionCoeff * this.mass * 9.81);
            this.force = PointCal.addVector(this.force, kineticFriction);
        }
        if (PointCal.magnitude(this.linearVelocity) < PointCal.magnitude(PointCal.divideVectorByScalar(PointCal.multiplyVectorByScalar(this.force, deltaTime), this.mass))){
            this.linearVelocity = {x: 0, y: 0};
        }
        this.linearVelocity = PointCal.addVector(this.linearVelocity, PointCal.divideVectorByScalar(PointCal.multiplyVectorByScalar(this.force, deltaTime), this.mass));
        // if (PointCal.magnitude(PointCal.subVector(this.linearVelocity, {x: 0, y: 0})) <= 0.05) {
        //     this.linearVelocity = {x: 0, y: 0};
        // }
        this.center = PointCal.addVector(this.center, PointCal.multiplyVectorByScalar(this.linearVelocity, deltaTime));
        this.force = {x: 0, y: 0};
    }

    abstract getMinMaxProjection(unitvector: Point): {min: number, max: number};
    abstract getCollisionAxes(relativeBody: RigidBody): Point[];
    abstract getAABB(): {min: Point, max: Point};

}

export class Polygon extends BaseRigidBody {

    private vertices: Point[];

    constructor(center: Point = {x: 0, y: 0}, vertices: Point[], orientationAngle: number = 0, mass: number = 50, isStatic: boolean = false, frictionEnabled: boolean = true) {
        super(center, orientationAngle, mass, isStatic, frictionEnabled);
        this.vertices = vertices;
        this.step = this.step.bind(this);
    }


    getVerticesAbsCoord(): Point[]{
        return this.vertices.map(vertex=>{
            return PointCal.addVector(this.center, PointCal.rotatePoint(vertex, this.orientationAngle));
        });
    }

    getCollisionAxes(relativeBody: RigidBody): Point[] {
        return this.getVerticesAbsCoord().map((vertex, index, absVertices)=>{
            let vector = PointCal.subVector(vertex, absVertices[absVertices.length - 1]);
            if (index > 0){
                vector = PointCal.subVector(vertex, absVertices[index - 1]); 
            }
            return PointCal.unitVector(PointCal.rotatePoint(vector, Math.PI / 2));
        });
    }

    getMinMaxProjection(unitvector: Point): { min: number; max: number; } {
        let vertices = this.getVerticesAbsCoord();
        
        let projections = vertices.map( vertex => {
            return PointCal.dotProduct(vertex, unitvector);
        });

        
        return {min: Math.min(...projections), max: Math.max(...projections)};
    }

    getAABB(): { min: Point; max: Point; } {
        let points = this.getVerticesAbsCoord();
        let xCoords = points.map(vertex => vertex.x);
        let yCoords = points.map(vertex => vertex.y);
        return {min: {x: Math.min(...xCoords), y: Math.min(...yCoords)}, max: {x: Math.max(...xCoords), y: Math.max(...yCoords)}};
    }

}