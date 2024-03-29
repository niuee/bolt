import typescript from '@rollup/plugin-typescript';
import resolve from '@rollup/plugin-node-resolve';
import dts from "rollup-plugin-dts";
import terser from "@rollup/plugin-terser";
import path from 'path';
const packageJson = require("./package.json");



const fs = require('fs');

const plugins = [
    resolve(),
    typescript({
      tsconfig: './tsconfig.json',
      declaration: false,
      // useTsconfigDeclarationDir: true,
    }),
    terser({
      mangle: false,
    }),
]

export const getComponentsFolders = (entry) => {
   const dirs = fs.readdirSync(entry)
   const dirsWithoutIndex = dirs.filter(name => name !== 'index.ts' && name !== 'utils')
   return dirsWithoutIndex
};

export const getComponentsFoldersRecursive = (entry) => {
  const finalListOfDirs = [];
  const dirs = fs.readdirSync(entry)
  while (dirs.length !== 0){
    const length = dirs.length;
    for(let i=0; i < length; i++){
      const dir = dirs.shift();
      if(fs.statSync(path.resolve(entry, dir)).isDirectory()){
        if (entry === './src') {
          finalListOfDirs.push(dir);   
        } else {
          finalListOfDirs.push(path.join(entry, dir));
        }
        const subDirs = fs.readdirSync(path.resolve(entry, dir));
        dirs.push(...subDirs.map(subDir => path.join(dir, subDir)));
      }
    }
  } 
  return finalListOfDirs;
};

console.log(getComponentsFoldersRecursive('./src'));


const folderBuilds = getComponentsFoldersRecursive('./src').map((folder) => {
  return {
    input: `src/${folder}/index.ts`,
    output: [
    {
      file: `build/${folder}/index.js`,
      sourcemap: true,
      format: 'esm',
    },
    ],
    plugins: [
        ...plugins,
    ]
  };
});


const types = getComponentsFoldersRecursive('./src').map((folder) => {
  return {
    input: `src/${folder}/index.ts`,
    output: {
      file: `build/${folder}/index.d.ts`,
      format: "es",
    },
    plugins: [
      dts.default(),
    ],
  };

});

export default [
  ...folderBuilds,
  {
    input: 'src/index.ts',
    output: [{
      file: packageJson.main,
      format: 'cjs',
      name: '@niuee/bolt',
      sourcemap: true,
    },  
    {
      file: packageJson.module,
      format: 'esm',
      name: '@niuee/bolt',
      sourcemap: true
    }
    ],
    plugins: [
      resolve(),
      typescript({
        tsconfig: './tsconfig.json',
        declaration: true,
        declarationDir: "./build",
      }),
      terser({
        mangle: false,
      }),
    ],
  },
  {
    // distribution for direct browser usage
    input: 'src/index.ts',
    output: [
    {
      file: 'dist/bolt.js',
      format: 'esm',
      name: 'board',
      sourcemap: true,
    },
    {
      file: 'build/umd/index.js',
      format: 'umd',
      name: "Board",
      sourcemap: true
    },
    {
      file: 'build/iife/index.js',
      format: 'iife',
      name: "Board",
      sourcemap: true
    }
    ],
    plugins: [
      resolve(),
      typescript({
        tsconfig: './tsconfig.json',
        declaration: false,
      }),
      terser({
        mangle: false,
      }),
    ],
  }
];