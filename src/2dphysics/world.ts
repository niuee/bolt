import { BaseRigidBody, RigidBody } from "./rigidbody";
import { Collision } from "./collision";
import { RectangleBound, QuadTree} from "./quadtree"

export class World {
    private rigidBodyList: BaseRigidBody[];
    private rigidBodyMap: Map<string, BaseRigidBody>;
    private resolveCollision: boolean;
    private maxTransWidth: number;
    private maxTransHeight: number;
    private bound: RectangleBound;
    private quadTree: QuadTree;

    constructor(maxTransWidth: number, maxTransHeight: number){
        this.maxTransHeight = maxTransHeight;
        this.maxTransWidth = maxTransWidth;
        this.bound = new RectangleBound({x: -this.maxTransWidth, y: -this.maxTransHeight}, 2 * this.maxTransWidth, 2 * this.maxTransHeight);
        this.quadTree = new QuadTree(0, this.bound);
        this.rigidBodyList = [];
        this.rigidBodyMap = new Map<string, BaseRigidBody>();
        this.resolveCollision = true;
    }

    addRigidBody(ident: string, body: BaseRigidBody): void{
        this.rigidBodyList.push(body);
        this.rigidBodyMap.set(ident, body);
    }

    removeRigidBody(ident: string): void{
        if (this.rigidBodyMap.has(ident)) {
            this.rigidBodyMap.delete(ident);
        }
    }

    step(deltaTime: number): void{
        // console.log("stepping in world");
        let rigidBodyList: BaseRigidBody[] = [];
        this.quadTree.clear();
        this.rigidBodyMap.forEach((body) => {
            rigidBodyList.push(body);
            this.quadTree.insert(body);
        });
        let possibleCombinations = Collision.broadPhase(this.quadTree, rigidBodyList);
        Collision.narrowPhase(rigidBodyList, possibleCombinations, this.resolveCollision);
        rigidBodyList.forEach(rigidBody => {
            rigidBody.step(deltaTime);
        })
    }

    getRigidBodyList(){
        let rigidBodyList:BaseRigidBody[] = [];
        this.rigidBodyMap.forEach((body) => {
            rigidBodyList.push(body);
        })
        return rigidBodyList;
    }

    getRigidBodyMap(): Map<string, BaseRigidBody>{
        return this.rigidBodyMap;
    }

    setMaxTransHeight(height: number){
        this.maxTransHeight = height;
    }

    setMaxTransWidth(width: number){
        this.maxTransWidth = width;
    }
}
