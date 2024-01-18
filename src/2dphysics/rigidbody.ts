import { PointCal, Point } from "point2point";

export interface RigidBody {
    center: Point;
    linearVelocity: Point;
    angularVelocity: number;
    step(deltaTime: number): void;
    isStatic(): boolean;
    isMovingStatic(): boolean;
    getMinMaxProjection(unitvector: Point): {min: number, max: number};
    getCollisionAxes(relativeBody: RigidBody): Point[];
    applyForce(force: Point): void;
    applyForceInOrientation(force: Point): void;
    getAABB(): {min: Point, max: Point};
    getMass(): number;
    move(delta: Point): void;
    significantVertex(collisionNormal: Point): Point;
}

export interface VisualComponent{
    draw(ctx: CanvasRenderingContext2D): void;
}



export abstract class BaseRigidBody implements RigidBody{
    
    protected _center: Point;
    protected orientationAngle: number = 0;
    protected mass: number = 50;
    protected _linearVelocity: Point;
    protected linearAcceleartion: Point;
    protected force: Point;
    protected isStaticBody: boolean = false;
    protected staticFrictionCoeff: number = 0.3;
    protected dynamicFrictionCoeff: number = 0.3;
    protected frictionEnabled: boolean = false;
    protected isMovingStaticBody: boolean = false;
    protected _angularVelocity: number; // in radians
    protected angularDampingFactor: number = 0.001;
    

    constructor(center: Point, orientationAngle: number = 0, mass: number = 50, isStaticBody: boolean = false, frictionEnabled: boolean = false){
        this._center = center;
        this.orientationAngle = orientationAngle;
        this.mass = mass;
        this.isStaticBody = isStaticBody;
        this.frictionEnabled = frictionEnabled;
        this.force = {x: 0, y: 0};
        this.linearAcceleartion = {x: 0, y: 0};
        this._linearVelocity = {x: 0, y: 0};
        this._angularVelocity = 0;
    }

    move(delta: Point): void {
        if (!this.isStatic()){
            this._center = PointCal.addVector(this._center, delta);
        }
    }

    rotateRadians(angle: number): void {
        this.orientationAngle += angle;
    }

    getCenter(): Point {
        return this._center;
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

    get angularVelocity(): number{
        return this._angularVelocity;
    }

    set angularVelocity(angularVelocity: number){
        this._angularVelocity = angularVelocity;
    }

    isStatic(): boolean{
        return this.isStaticBody;
    }

    isMovingStatic(): boolean {
        return this.isMovingStaticBody;
    }

    setLinearVelocity(linearVelocity: Point): void {
        this._linearVelocity = linearVelocity;
    }

    setMovingStatic(movingStatic: boolean):void {
        this.isMovingStaticBody = movingStatic;
    }

    setOrientationAngle(angle: number): void{
        this.orientationAngle = angle;
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
                // return;
            } else {
                let kineticFrictionDirection = PointCal.multiplyVectorByScalar(PointCal.unitVector(this._linearVelocity), -1);
                let kineticFriction = PointCal.multiplyVectorByScalar(kineticFrictionDirection, this.dynamicFrictionCoeff * this.mass * 9.81);
                this.force = PointCal.addVector(this.force, kineticFriction);
            }
        }
        const angularDamping = this._angularVelocity != 0 ? this._angularVelocity * (-1 / this._angularVelocity) * this.angularDampingFactor : 0;
        // console.log("angular velocity", this._angularVelocity);
        // console.log("angular damping", angularDamping);
        if (Math.abs(this._angularVelocity) < Math.abs(angularDamping)) {
            this._angularVelocity = 0;
        } else {
            this._angularVelocity += angularDamping;
        }
        this.orientationAngle += this._angularVelocity * deltaTime;
        if (PointCal.magnitude(this._linearVelocity) < PointCal.magnitude(PointCal.divideVectorByScalar(PointCal.multiplyVectorByScalar(this.force, deltaTime), this.mass))){
            this._linearVelocity = {x: 0, y: 0};
        }
        this._linearVelocity = PointCal.addVector(this._linearVelocity, PointCal.divideVectorByScalar(PointCal.multiplyVectorByScalar(this.force, deltaTime), this.mass));
        this._center = PointCal.addVector(this._center, PointCal.multiplyVectorByScalar(this._linearVelocity, deltaTime));
        this.force = {x: 0, y: 0};
    }

    get center(): Point {
        return this._center;
    }

    set center(dest: Point){
        this._center = dest;
    }

    get linearVelocity(): Point {
        return this._linearVelocity;
    }

    set linearVelocity(dest: Point){
        this._linearVelocity = dest;
    }

    abstract getMinMaxProjection(unitvector: Point): {min: number, max: number};
    abstract getCollisionAxes(relativeBody: RigidBody): Point[];
    abstract getAABB(): {min: Point, max: Point};
    abstract significantVertex(collisionNormal: Point): Point;

}

export class Circle extends BaseRigidBody {

    private _radius: number;
    constructor(center: Point = {x: 0, y: 0}, radius: number, orientationAngle: number = 0, mass: number = 50, isStatic: boolean = false, frictionEnabled: boolean = true) {
        super(center, orientationAngle, mass, isStatic, frictionEnabled);
        this._radius = radius;
        this.step = this.step.bind(this);
    }

    getMinMaxProjection(unitvector: Point): { min: number; max: number; } {
        let PositiveFurthest = PointCal.addVector(this._center, PointCal.multiplyVectorByScalar(unitvector, this._radius));
        let NegativeFurthest = PointCal.addVector(this._center, PointCal.multiplyVectorByScalar(unitvector, -this._radius));
        return {min: PointCal.dotProduct(NegativeFurthest, unitvector), max: PointCal.dotProduct(PositiveFurthest, unitvector)};
    }

    getCollisionAxes(relativeBody: RigidBody): Point[] {
        return [PointCal.unitVector(PointCal.subVector(relativeBody.center, this._center))];
    }

    getAABB(): { min: Point; max: Point; } {
        return {min: PointCal.subVector(this._center, {x: this._radius, y: this._radius}), max: PointCal.addVector(this._center, {x: this._radius, y: this._radius})};
    }

    significantVertex(collisionNormal: Point): Point {
        return PointCal.addVector(this._center, PointCal.multiplyVectorByScalar(collisionNormal, this._radius));
    }

    get radius(): number {
        return this._radius;
    }
}

export class VisaulCircleBody implements VisualComponent, RigidBody {

    private _circle: Circle;
    private _context: CanvasRenderingContext2D;

    constructor(center: Point = {x: 0, y: 0}, radius: number, drawingContext: CanvasRenderingContext2D, orientationAngle: number = 0, mass: number = 50, isStatic: boolean = false, frictionEnabled: boolean = true) {
        this._circle = new Circle(center, radius, orientationAngle, mass, isStatic, frictionEnabled);
        this._context = drawingContext;
    }

    draw(ctx: CanvasRenderingContext2D): void {
        ctx.beginPath();
        ctx.arc(this._circle.center.x, -this._circle.center.y, this._circle.radius, 0, 2 * Math.PI);
        ctx.stroke();
    }

    step(deltaTime: number): void {
        this._circle.step(deltaTime);
        this.draw(this._context);
    }

    isStatic(): boolean {
        return this._circle.isStatic();
    }

    isMovingStatic(): boolean {
        return this._circle.isMovingStatic();
    }

    getMinMaxProjection(unitvector: Point): { min: number; max: number; } {
        return this._circle.getMinMaxProjection(unitvector);
    }

    getCollisionAxes(relativeBody: RigidBody): Point[] {
        return this._circle.getCollisionAxes(relativeBody);
    }

    applyForce(force: Point): void {
        this._circle.applyForce(force);
    }

    getAABB(): { min: Point; max: Point; } {
        return this._circle.getAABB();
    }

    getMass(): number {
        return this._circle.getMass();
    }

    getLinearVelocity(): Point {
        return this._circle.getLinearVelocity();
    }

    applyForceInOrientation(force: Point): void {
        this._circle.applyForceInOrientation(force);
    }

    setLinearVelocity(linearVelocity: Point): void {
        this._circle.setLinearVelocity(linearVelocity);
    }

    move(delta: Point): void {
        this._circle.move(delta);
    }

    get center(): Point {
        return this._circle.center;
    }

    set center(dest: Point){
        this._circle.center = dest;
    }

    get linearVelocity(): Point {
        return this._circle.linearVelocity;
    }

    set linearVelocity(dest: Point){
        this._circle.linearVelocity = dest;
    }

    significantVertex(collisionNormal: Point): Point {
        return this._circle.significantVertex(collisionNormal);
    }

    set angularVelocity(angularVelocity: number){
        this._circle.angularVelocity = angularVelocity;
    }

    get angularVelocity(): number{
        return this._circle.angularVelocity;
    }

}

export class VisualPolygonBody implements VisualComponent, RigidBody {
    
    private _polygon: Polygon;
    private _context: CanvasRenderingContext2D;

    constructor(center: Point = {x: 0, y: 0}, vertices: Point[], drawingContext: CanvasRenderingContext2D, orientationAngle: number = 0, mass: number = 50, isStatic: boolean = false, frictionEnabled: boolean = true) {
        this._polygon = new Polygon(center, vertices, orientationAngle, mass, isStatic, frictionEnabled);
        this._context = drawingContext;
    }

    draw(ctx: CanvasRenderingContext2D): void {
        ctx.beginPath();
        let vertices = this._polygon.getVerticesAbsCoord();
        ctx.moveTo(vertices[0].x, -vertices[0].y);
        vertices.forEach(vertex => {
            ctx.lineTo(vertex.x, -vertex.y);
        });
        ctx.lineTo(vertices[0].x, -vertices[0].y);
        ctx.stroke();
    }

    step(deltaTime: number): void {
        this._polygon.step(deltaTime);
        this.draw(this._context);
    }

    isStatic(): boolean {
        return this._polygon.isStatic();
    }

    isMovingStatic(): boolean {
        return this._polygon.isMovingStatic();
    }

    getMinMaxProjection(unitvector: Point): { min: number; max: number; } {
        return this._polygon.getMinMaxProjection(unitvector);
    }

    getCollisionAxes(relativeBody: RigidBody): Point[] {
        return this._polygon.getCollisionAxes(relativeBody);
    }

    applyForce(force: Point): void {
        this._polygon.applyForce(force);
    }

    getAABB(): { min: Point; max: Point; } {
        return this._polygon.getAABB();
    }

    getMass(): number {
        return this._polygon.getMass();
    }

    getLinearVelocity(): Point {
        return this._polygon.getLinearVelocity();
    }

    applyForceInOrientation(force: Point): void {
        this._polygon.applyForceInOrientation(force);
    }

    setLinearVelocity(linearVelocity: Point): void {
        this._polygon.setLinearVelocity(linearVelocity);
    }

    move(delta: Point): void {
        this._polygon.move(delta);
    }

    get center(): Point {
        return this._polygon.center;
    }

    set center(dest: Point){
        this._polygon.center = dest;
    }

    get linearVelocity(): Point {
        return this._polygon.linearVelocity;
    }

    set linearVelocity(dest: Point){
        this._polygon.linearVelocity = dest;
    }

    get angularVelocity(): number{
        return this._polygon.angularVelocity;
    }

    set angularVelocity(angularVelocity: number){
        this._polygon.angularVelocity = angularVelocity;
    }

    significantVertex(collisionNormal: Point): Point {
        return this._polygon.significantVertex(collisionNormal);
    }

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
            return PointCal.addVector(this._center, PointCal.rotatePoint(vertex, this.orientationAngle));
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

    significantVertex(collisionNormal: Point): Point {
        let vertices = this.getVerticesAbsCoord();
        let verticesProjected = vertices.map(vertex => PointCal.dotProduct(vertex, collisionNormal));
        let maxIndex = verticesProjected.indexOf(Math.max(...verticesProjected));
        return vertices[maxIndex];
    }

}